idf_component_register(
    SRCS
        "p1p2_fseries_decode.c"
        "p1p2_fseries_control.c"
        "p1p2_param_conversion.c"
    INCLUDE_DIRS "include"
    REQUIRES
        p1p2_bus
)
