#
# 86Box    A hypervisor and IBM PC system emulator that specializes in
#          running old operating systems and software designed for IBM
#          PC systems and compatibles from 1981 through fairly recent
#          system designs based on the PCI bus.
#
#          This file is part of the 86Box distribution.
#
#          CMake build script.
#
# Authors: David Hrdlička, <hrdlickadavid@outlook.com>
#          RichardG, <richardg867@gmail.com>
#
#          Copyright 2020-2023 David Hrdlička.
#          Copyright 2021-2025 RichardG.
#

if(CLI)
    add_library(cli OBJECT
        cli.c
        cli_input.c
        cli_monitor.c
        cli_render.c
    )
    target_link_libraries(86Box cli)
endif()
