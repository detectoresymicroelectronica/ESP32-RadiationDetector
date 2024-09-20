# get IDF version for comparison
set(idf_version "${IDF_VERSION_MAJOR}.${IDF_VERSION_MINOR}")

set(COMPONENT_ADD_INCLUDEDIRS
  Librerias/include
  )

set(COMPONENT_REQUIRES driver)

# set driver sources only for supported platforms
if(IDF_TARGET STREQUAL "esp32")
  list(APPEND COMPONENT_SRCS
    Librerias/esp_camera.c
    Librerias/cam_hal.c
    Librerias/sccb.c
    Librerias/sensor.c
    Librerias/ov2640.c
    Librerias/xclk.c
    Librerias/ll_cam.c
    )

  list(APPEND COMPONENT_PRIV_INCLUDEDIRS
    Librerias/private_include
    )

  set(COMPONENT_PRIV_REQUIRES freertos nvs_flash)

  set(min_version_for_esp_timer "4.2")
  if (idf_version VERSION_GREATER_EQUAL min_version_for_esp_timer)
    list(APPEND COMPONENT_PRIV_REQUIRES esp_timer)
  endif()

endif()

register_component()
