idf_component_register(
    SRCS
        "p1p2_cli.c"
    INCLUDE_DIRS "."
    REQUIRES
        p1p2_bus
        p1p2_protocol
        p1p2_network
        console
        driver
)
