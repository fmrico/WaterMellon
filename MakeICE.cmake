set(BICA_ICEFILES_DIR ${CMAKE_CURRENT_LIST_DIR}/ICE)
set(BICA_CPP_OUTPUT_DIR ${CMAKE_CURRENT_LIST_DIR}/src/wm_bica)
set(BICA_H_OUTPUT_DIR ${CMAKE_CURRENT_LIST_DIR}/include/wm_bica)

set(SLICE2CPP slice2cpp)

#file(MAKE_DIRECTORY ${BICA_OUTPUT_DIR})

set (BICA_FILES
    BicaIceComms.ice
)

foreach (file ${BICA_FILES})
 	set (bica_file ${file})
	GET_FILENAME_COMPONENT (bica_file_name ${bica_file} NAME_WE)
	
	set(args --include-dir ${BICA_H_OUTPUT_DIR} --output-dir=${BICA_CPP_OUTPUT_DIR} -I. -I${BICA_ICEFILES_DIR} ${BICA_ICEFILES_DIR}/${bica_file})		

	ADD_CUSTOM_COMMAND (
 		OUTPUT ${BICA_CPP_OUTPUT_DIR}/${bica_file_name}.cpp ${BICA_H_OUTPUT_DIR}/${bica_file_name}.h
  		DEPENDS ${BICA_ICEFILES_DIR}/${bica_file}
  		COMMAND ${SLICE2CPP} ${args}
  	)
  	
  	execute_process(COMMAND ${SLICE2CPP} ${args}) 
  	execute_process(COMMAND mv ${BICA_CPP_OUTPUT_DIR}/${bica_file_name}.h ${BICA_H_OUTPUT_DIR}) 
  	
  	#-> ${BICA_OUTPUT_DIR}/${bica_file_name}.cpp ${BICA_OUTPUT_DIR}/${bica_file_name}.h")
  	
    #set (bica_file ${file}.ice)
    #set(args --output-dir=${BICA_OUTPUT_DIR} -I. -I${BICA_ICEFILES_DIR} ${BICA_ICEFILES_DIR}/${bica_file})
    #execute_process(COMMAND ${SLICE2CPP} ${args})   
endforeach()

