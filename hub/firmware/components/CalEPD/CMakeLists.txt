# This file is based on source code originally from martinberlin/CalEPD GitHub repository, 
# available at https://github.com/martinberlin/CalEPD.
# 
# Modifications have been made to the original code by Jakub Franek (https://github.com/JakubFranek), 
# as permitted under the Apache License, Version 2.0.

set(srcs 
    
    #"src/displays/goodisplay/gdey029T94.cpp"
    "src/displays/goodisplay/gdey075T7.cpp"

    "src/epd.cpp"
    "src/epdspi.cpp"
    )


idf_component_register(SRCS ${srcs}      
                    REQUIRES "Adafruit-GFX"
                    REQUIRES esp_timer 
                             driver

                    INCLUDE_DIRS "include"
)