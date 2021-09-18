set(COMPONENT_SRCS "main.c" "ui.c" "fft.c" "wifi.c" "sterilizeRobot.c"  "ntptime.c" "../components/dc-motor/m5-dcmotor.c" )
set(COMPONENT_ADD_INCLUDEDIRS "." "./includes" "../components/dc-motor")

register_component()

target_add_binary_data(${COMPONENT_TARGET} "certs/aws-root-ca.pem" TEXT)