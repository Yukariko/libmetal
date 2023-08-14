collect (PROJECT_LIB_HEADERS sys.h)

collect (PROJECT_LIB_SOURCES irq.c)

if ("${PROJECT_MACHINE}" STREQUAL "zynqmp_a53" OR "${PROJECT_MACHINE}" STREQUAL "zynqmp_a72" OR "${PROJECT_MACHINE}" STREQUAL "zynqmp_a78")
  add_subdirectory(zynqmp_aarch64)
endif ("${PROJECT_MACHINE}" STREQUAL "zynqmp_a53" OR "${PROJECT_MACHINE}" STREQUAL "zynqmp_a72" OR "${PROJECT_MACHINE}" STREQUAL "zynqmp_a78")

# vim: expandtab:ts=2:sw=2:smartindent
