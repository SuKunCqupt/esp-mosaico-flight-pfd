idf_component_register(SRCS "pfd_gfx.c" "pfd_view.c" "pfd_font_data.c" "qrcodegen.c"
    INCLUDE_DIRS "include"
    PRIV_INCLUDE_DIRS ".")
target_compile_options(${COMPONENT_LIB} PRIVATE -O2)
