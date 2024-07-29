use core::arch::global_asm;
use core::marker::PhantomData;
use spin::Mutex;

use axvcpu::AxArchVCpuExitReason;
use cortex_a::registers::*;
use tock_registers::interfaces::*;

use super::context_frame::VmContext;
use super::exception_utils::*;
use super::register_lower_aarch64_synchronous_handler_arch;
use super::sync::{data_abort_handler, hvc_handler};
use super::ContextFrame;
use axerrno::{AxError, AxResult};

use crate::{AxVMHal, GuestPhysAddr, HostPhysAddr};

global_asm!(include_str!("entry.S"));
global_asm!(include_str!("exception.S"));

// TSC, bit [19]
const HCR_TSC_TRAP: usize = 1 << 19;

/// (v)CPU register state that must be saved or restored when entering/exiting a VM or switching
/// between VMs.
#[repr(C)]
#[derive(Clone, Debug, Copy, Default)]
pub struct VmCpuRegisters {
    /// guest trap context
    pub trap_context_regs: ContextFrame,
    /// virtual machine system regs setting
    pub vm_system_regs: VmContext,
}

impl VmCpuRegisters {
    /// create a default VmCpuRegisters
    pub fn default() -> VmCpuRegisters {
        VmCpuRegisters {
            trap_context_regs: ContextFrame::default(),
            vm_system_regs: VmContext::default(),
        }
    }
}

/// A virtual CPU within a guest
#[derive(Clone, Debug)]
pub struct VCpu<H: AxVMHal> {
    /// Vcpu context
    ctx: ContextFrame,
    host_stack_top: u64,
    system_regs: VmContext,
    vcpu_id: usize,

    marker: PhantomData<H>,
}

extern "C" {
    fn context_vm_entry(ctx: usize);
    fn save_context();
    fn restore_context();
}

pub type AxArchVCpuConfig = VmCpuRegisters;

impl<H: AxVMHal> axvcpu::AxArchVCpu for VCpu<H> {
    type CreateConfig = ();

    type SetupConfig = ();

    fn new(_config: Self::CreateConfig) -> AxResult<Self> {
        Ok(Self {
            ctx: ContextFrame::default(),
            host_stack_top: 0,
            system_regs: VmContext::default(),
            vcpu_id: 0, // need to pass a parameter!!!!
            marker: PhantomData,
        })
    }

    fn setup(&mut self, _config: Self::SetupConfig) -> AxResult {
        register_lower_aarch64_synchronous_handler_arch()?;
        self.init_hv();
        Ok(())
    }

    fn set_entry(&mut self, entry: GuestPhysAddr) -> AxResult {
        debug!("set vcpu entry:{:#x}", entry);
        self.set_elr(entry);
        Ok(())
    }

    fn set_ept_root(&mut self, ept_root: HostPhysAddr) -> AxResult {
        debug!("set vcpu ept root:{:#x}", ept_root);
        self.system_regs.vttbr_el2 = ept_root.as_usize() as u64;
        Ok(())
    }
    
    fn run(&mut self) -> AxResult<AxArchVCpuExitReason> {
        mark();
        self.run_guest();
        self.vmexit_handler()
    }

    fn bind(&mut self) -> AxResult {
        Ok(())
    }

    fn unbind(&mut self) -> AxResult {
        Ok(())
    }
}

#[no_mangle]
#[inline(never)]
fn mark() {
    debug!("mark");
}

// Private function
impl<H: AxVMHal> VCpu<H> {
    #[inline(never)]
    fn run_guest(&mut self) {
        unsafe {
            // load system regs
            core::arch::asm!(
                "
                mov x3, xzr           // Trap nothing from EL1 to El2.
                msr cptr_el2, x3"
            );
            self.system_regs.ext_regs_restore();
            cache_invalidate(0 << 1);
            cache_invalidate(1 << 1);
            core::arch::asm!(
                "
                ic  iallu
                tlbi	alle2
                tlbi	alle1         // Flush tlb
                dsb	nsh
                isb"
            );

            core::arch::asm!(
                "bl save_context",  // save host context
                "mov x9, sp",
                "mov x10, {0}",
                "str x9, [x10]",    // save host stack top in the vcpu struct
                "mov x0, {0}",
                "b context_vm_entry",
                in(reg) &self.host_stack_top as *const _ as usize,
                options(nostack)
            );
            // context_vm_entry(&self.host_stack_top as *const _ as usize);
        }
    }

    fn vmexit_handler(&mut self) -> AxResult<AxArchVCpuExitReason> {
        debug!(
            "enter lower_aarch64_synchronous exception class:0x{:X}",
            exception_class()
        );
        // save system regs
        self.system_regs.ext_regs_store();
        
        let ctx = &mut self.ctx;
        match exception_class() {
            0x24 => return data_abort_handler(ctx),
            0x16 => return hvc_handler(ctx),
            // 0x18 todo？
            _ => {
                panic!(
                    "handler not presents for EC_{} @ipa 0x{:x}, @pc 0x{:x}, @esr 0x{:x}, @sctlr_el1 0x{:x}, @vttbr_el2 0x{:x}, @vtcr_el2: {:#x} hcr: {:#x} ctx:{}",
                    exception_class(),
                    exception_fault_addr(),
                    (*ctx).exception_pc(),
                    exception_esr(),
                    cortex_a::registers::SCTLR_EL1.get() as usize,
                    cortex_a::registers::VTTBR_EL2.get() as usize,
                    cortex_a::registers::VTCR_EL2.get() as usize,
                    cortex_a::registers::HCR_EL2.get() as usize,
                    ctx
                );
            }
        }
    }

    fn init_hv(&mut self) {
        self.ctx.spsr = (SPSR_EL1::M::EL1h
            + SPSR_EL1::I::Masked
            + SPSR_EL1::F::Masked
            + SPSR_EL1::A::Masked
            + SPSR_EL1::D::Masked)
            .value;
        self.init_vm_context();
    }

    /// Init guest context. Also set some el2 register value.
    fn init_vm_context(&mut self) {
        CNTHCTL_EL2.modify(CNTHCTL_EL2::EL1PCEN::SET + CNTHCTL_EL2::EL1PCTEN::SET);
        self.system_regs.cntvoff_el2 = 0;
        self.system_regs.cntkctl_el1 = 0;

        self.system_regs.sctlr_el1 = 0x30C50830;
        self.system_regs.pmcr_el0 = 0;
        self.system_regs.vtcr_el2 = (VTCR_EL2::PS::PA_40B_1TB
            + VTCR_EL2::TG0::Granule4KB
            + VTCR_EL2::SH0::Inner
            + VTCR_EL2::ORGN0::NormalWBRAWA
            + VTCR_EL2::IRGN0::NormalWBRAWA
            + VTCR_EL2::SL0.val(0b01)
            + VTCR_EL2::T0SZ.val(64 - 39))
        .into();
        self.system_regs.hcr_el2 = (HCR_EL2::VM::Enable + HCR_EL2::RW::EL1IsAarch64).into();
        // self.system_regs.hcr_el2 |= 1<<27;
        // + HCR_EL2::IMO::EnableVirtualIRQ).into();
        // trap el1 smc to el2
        // self.system_regs.hcr_el2 |= HCR_TSC_TRAP as u64;

        let mut vmpidr = 0;
        vmpidr |= 1 << 31;
        vmpidr |= self.vcpu_id;
        self.system_regs.vmpidr_el2 = vmpidr as u64;
    }

    /// Set exception return pc
    fn set_elr(&mut self, elr: usize) {
        self.ctx.set_exception_pc(elr);
    }

    /// Get general purpose register
    fn get_gpr(&mut self, idx: usize) {
        self.ctx.gpr(idx);
    }

    /// Set general purpose register
    fn set_gpr(&mut self, idx: usize, val: usize) {
        self.ctx.set_gpr(idx, val);
    }
}

#[naked]
pub unsafe extern "C" fn vmexit_aarch64_handler() {
    // save guest context
    core::arch::asm!(
        "bl save_context", // save guest context
        "mov x9, sp",
        "ldr x10, [x9]",
        "mov sp, x10",        // move sp to the host stack top value
        "bl restore_context", // restore host context
        "ret",
        options(noreturn),
    )
}

unsafe fn cache_invalidate(cache_level: usize) {
    core::arch::asm!(
        r#"
        msr csselr_el1, {0}
        mrs x4, ccsidr_el1 // read cache size id.
        and x0, x4, #0x7
        add x0, x0, #0x4 // x0 = cache line size.
        ldr x3, =0x7fff
        and x2, x3, x4, lsr #13 // x2 = cache set number – 1.
        ldr x3, =0x3ff
        and x3, x3, x4, lsr #3 // x3 = cache associativity number – 1.
        clz w4, w3 // x4 = way position in the cisw instruction.
        mov x5, #0 // x5 = way counter way_loop.
    // way_loop:
    1:
        mov x6, #0 // x6 = set counter set_loop.
    // set_loop:
    2:
        lsl x7, x5, x4
        orr x7, {0}, x7 // set way.
        lsl x8, x6, x0
        orr x7, x7, x8 // set set.
        dc csw, x7 // clean and invalidate cache line.
        add x6, x6, #1 // increment set counter.
        cmp x6, x2 // last set reached yet?
        ble 2b // if not, iterate set_loop,
        add x5, x5, #1 // else, next way.
        cmp x5, x3 // last way reached yet?
        ble 1b // if not, iterate way_loop
        "#,
        in(reg) cache_level,
        options(nostack)
    );
}
