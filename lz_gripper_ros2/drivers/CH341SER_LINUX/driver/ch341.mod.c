#include <linux/module.h>
#define INCLUDE_VERMAGIC
#include <linux/build-salt.h>
#include <linux/elfnote-lto.h>
#include <linux/export-internal.h>
#include <linux/vermagic.h>
#include <linux/compiler.h>

#ifdef CONFIG_UNWINDER_ORC
#include <asm/orc_header.h>
ORC_HEADER;
#endif

BUILD_SALT;
BUILD_LTO_INFO;

MODULE_INFO(vermagic, VERMAGIC_STRING);
MODULE_INFO(name, KBUILD_MODNAME);

__visible struct module __this_module
__section(".gnu.linkonce.this_module") = {
	.name = KBUILD_MODNAME,
	.init = init_module,
#ifdef CONFIG_MODULE_UNLOAD
	.exit = cleanup_module,
#endif
	.arch = MODULE_ARCH_INIT,
};

#ifdef CONFIG_RETPOLINE
MODULE_INFO(retpoline, "Y");
#endif



static const struct modversion_info ____versions[]
__used __section("__versions") = {
	{ 0x8e17b3ae, "idr_destroy" },
	{ 0x54b1fac6, "__ubsan_handle_load_invalid_value" },
	{ 0x69acdf38, "memcpy" },
	{ 0xa1e2e28b, "usb_autopm_get_interface_async" },
	{ 0x1ac3e15, "usb_anchor_urb" },
	{ 0x839fd8bd, "usb_autopm_get_interface" },
	{ 0xf9c6fca3, "usb_control_msg" },
	{ 0x4c03a563, "random_kmalloc_seed" },
	{ 0xcb742157, "kmalloc_caches" },
	{ 0xfe1d3f1a, "kmalloc_trace" },
	{ 0x37a0cba, "kfree" },
	{ 0xc9195f47, "usb_ifnum_to_if" },
	{ 0x4dfa8d4b, "mutex_lock" },
	{ 0xb8f11603, "idr_alloc" },
	{ 0x3213f038, "mutex_unlock" },
	{ 0xd9a5ea54, "__init_waitqueue_head" },
	{ 0xcefb0c9f, "__mutex_init" },
	{ 0x58a5e659, "tty_port_init" },
	{ 0x7f993d76, "usb_alloc_coherent" },
	{ 0x86b5950e, "usb_alloc_urb" },
	{ 0x7665a95b, "idr_remove" },
	{ 0xf159e76d, "usb_free_coherent" },
	{ 0x41c99599, "_dev_info" },
	{ 0x40223a35, "usb_driver_claim_interface" },
	{ 0x70c73d27, "usb_get_intf" },
	{ 0x13bfdbf9, "tty_port_register_device" },
	{ 0x9ad1f3c7, "usb_free_urb" },
	{ 0xa54d561b, "__tty_insert_flip_string_flags" },
	{ 0x14a4433e, "tty_flip_buffer_push" },
	{ 0xf0fdf6cb, "__stack_chk_fail" },
	{ 0x20978fb9, "idr_find" },
	{ 0x6a88e1f3, "tty_standard_install" },
	{ 0x296695f, "refcount_warn_saturate" },
	{ 0x2d3385d3, "system_wq" },
	{ 0xc5b6f236, "queue_work_on" },
	{ 0x13c49cc2, "_copy_from_user" },
	{ 0xc6cbbc89, "capable" },
	{ 0x6b10bee1, "_copy_to_user" },
	{ 0xaad8c7d6, "default_wake_function" },
	{ 0xe2491b52, "pcpu_hot" },
	{ 0x4afb2238, "add_wait_queue" },
	{ 0x1000e51, "schedule" },
	{ 0x37110088, "remove_wait_queue" },
	{ 0xcd9c13a3, "tty_termios_hw_change" },
	{ 0xbd394d8, "tty_termios_baud_rate" },
	{ 0xaa14a21e, "usb_put_intf" },
	{ 0xf8c7291e, "tty_port_tty_get" },
	{ 0x4cc48f7d, "tty_vhangup" },
	{ 0xe202fd60, "tty_kref_put" },
	{ 0xd9f1b350, "tty_unregister_device" },
	{ 0x3cf9d87b, "usb_driver_release_interface" },
	{ 0xbdfb6dbb, "__fentry__" },
	{ 0x34db050b, "_raw_spin_lock_irqsave" },
	{ 0xd35cce70, "_raw_spin_unlock_irqrestore" },
	{ 0x5b8239ca, "__x86_return_thunk" },
	{ 0xbc1070b0, "usb_submit_urb" },
	{ 0x96dcc326, "_dev_err" },
	{ 0x87a21cb3, "__ubsan_handle_out_of_bounds" },
	{ 0x5a2ecddd, "usb_autopm_put_interface_async" },
	{ 0xbd514f86, "usb_kill_urb" },
	{ 0x3c12dfe, "cancel_work_sync" },
	{ 0x8427cc7b, "_raw_spin_lock_irq" },
	{ 0x4b750f53, "_raw_spin_unlock_irq" },
	{ 0xe6cc912d, "tty_port_put" },
	{ 0x18235fa6, "__tty_alloc_driver" },
	{ 0x67b27ec1, "tty_std_termios" },
	{ 0x146d804a, "tty_register_driver" },
	{ 0x3d4ed8ba, "usb_register_driver" },
	{ 0x122c3a7e, "_printk" },
	{ 0x22f35540, "tty_unregister_driver" },
	{ 0x5a1dd9ac, "tty_driver_kref_put" },
	{ 0x6ebe366f, "ktime_get_mono_fast_ns" },
	{ 0xe2964344, "__wake_up" },
	{ 0x4e5825e9, "__dynamic_dev_dbg" },
	{ 0x7393bc27, "tty_port_tty_hangup" },
	{ 0x963dd901, "usb_autopm_get_interface_no_resume" },
	{ 0xdf734fc6, "usb_autopm_put_interface" },
	{ 0x40c0fbdc, "usb_get_from_anchor" },
	{ 0xd424ec13, "tty_port_tty_wakeup" },
	{ 0xf00ec8c5, "tty_port_hangup" },
	{ 0x83c29275, "tty_port_close" },
	{ 0x3f66e4da, "tty_port_open" },
	{ 0x904f8725, "usb_deregister" },
	{ 0x8d5e53af, "module_layout" },
};

MODULE_INFO(depends, "");

MODULE_ALIAS("usb:v1A86p7523d*dc*dsc*dp*ic*isc*ip*in*");
MODULE_ALIAS("usb:v1A86p7522d*dc*dsc*dp*ic*isc*ip*in*");
MODULE_ALIAS("usb:v1A86p5523d*dc*dsc*dp*ic*isc*ip*in*");
MODULE_ALIAS("usb:v1A86pE523d*dc*dsc*dp*ic*isc*ip*in*");
MODULE_ALIAS("usb:v4348p5523d*dc*dsc*dp*ic*isc*ip*in*");

MODULE_INFO(srcversion, "8B035E6078723C06DFB968D");
