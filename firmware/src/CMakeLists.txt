idf_component_register(SRCS "main.cpp"
                            "network_core.cpp"
                            "mqtt_core.cpp"
                            "security_system.cpp"
                            "environment_system.cpp"
                     INCLUDE_DIRS "../include"
                     REQUIRES mdns)