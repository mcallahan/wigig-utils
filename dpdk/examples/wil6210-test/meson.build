executable('wiltest',
	'main.c',
	c_args: ['-O3', '-Werror'],
	link_whole: rte_pmd_wil6210,
	dependencies: [dependency('libdpdk', static: true, required: true)],
	install: true)
