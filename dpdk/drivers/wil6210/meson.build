
wil6210_sources = [
	'wil6210_api.c',
	'wil6210_compat.c',
	'wil6210_control.c',
	'wil6210_crash_dump.c',
	'wil6210_debug.c',
	'wil6210_ethdev.c',
	'wil6210_fw.c',
	'wil6210_fw_trace.c',
	'wil6210_interrupt.c',
	'wil6210_ioctl.c',
	'wil6210_main.c',
	'wil6210_memio.c',
	'wil6210_netdev.c',
	'wil6210_nl60g.c',
	'wil6210_pcidev.c',
	'wil6210_platform.c',
	'wil6210_platform_dpdk.c',
	'wil6210_pmc.c',
	'wil6210_rx_reorder.c',
	'wil6210_slave.c',
	'wil6210_txrx.c',
	'wil6210_txrx_edma.c',
	'wil6210_wmi.c',
	'dpdk-dhd-ctrl/dpdk-dhd-ctrl.c',
]

rte_pmd_wil6210 = static_library('rte_pmd_wil6210',
	wil6210_sources,
	c_args: [
		'-D_GNU_SOURCE',
		'-DWIL6210_PMD',
		'-DCONFIG_WIL6210_ISR_COR',
		'-Wno-deprecated',
		'-Wno-unused-parameter',
		'-Wno-unused-value',
		'-Wno-strict-aliasing',
		'-Wno-format-extra-args',
		'-Wno-pointer-arith',
		'-Wno-sign-compare'
	],
	include_directories: include_directories('wil6210', 'dpdk-dhd-ctrl', '../../modules/dpdk-dhd'),
	dependencies: [ dependency('libnl-3.0'), dependency('libnl-genl-3.0') ],
	link_args: '--version-script=rte_pmd_wil6210_version.map',
	install: true
)

install_headers('rte_wigig_api.h')
