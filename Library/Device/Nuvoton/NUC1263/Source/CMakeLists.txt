enable_language(C ASM)
add_library(system_startup 
    system_nuc1263.c 
    ARM/startup_nuc1263.s
)
