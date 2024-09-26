add_subdirectory(zynqGrabber)
add_subdirectory(hexViewer)

#add_subdirectory(binaryDumper)
#add_subdirectory(qadIMUcal)
#add_subdirectory(esim-yarp)

if(OpenCV_FOUND)
  add_subdirectory(vFramer)
  add_subdirectory(vPreProcess)
  add_subdirectory(calibration)
  add_subdirectory(log2vid)
endif()

if(prophesee_core_FOUND OR MetavisionSDK_FOUND)
  add_subdirectory(atis3-bridge)
endif()
