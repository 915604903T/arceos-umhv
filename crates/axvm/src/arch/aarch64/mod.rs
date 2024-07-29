mod context_frame;
pub mod device_list;
mod ept;
#[macro_use]
mod exception_utils;
mod hvc;
mod pcpu;
mod sync;
mod vcpu;

use spin::once::Once;

use axhal::arch::register_lower_aarch64_synchronous_handler;

pub use self::device_list::AxArchDeviceList;
pub use self::pcpu::PerCpu as AxVMArchPerCpuImpl;
pub use self::vcpu::VCpu as AxArchVCpuImpl;
pub use vcpu::AxArchVCpuConfig;

pub use self::ept::NestedPageTable as A64PageTable;
use axerrno::AxResult;

/// context frame for aarch64
pub type ContextFrame = context_frame::Aarch64ContextFrame;

pub fn has_hardware_support() -> bool {
    true
}

static INIT: Once = Once::new();

pub fn register_lower_aarch64_synchronous_handler_arch() -> AxResult {
    unsafe {
        INIT.call_once(|| {
            register_lower_aarch64_synchronous_handler(self::vcpu::vmexit_aarch64_handler)
        });
    }
    return Ok(());
}
