macro(lsdir result curdir)
	file(GLOB children RELATIVE ${curdir} ${curdir}/*)
	# file(GLOB children ${curdir}/*)
	message("children ${children}")
	set(dirlist "")
	foreach(child ${children})
		message("scan ${child}")
		if(IS_DIRECTORY ${curdir}/${child})
			message("append ${child}")
			list(APPEND dirlist ${child})
		endif()
	endforeach()
	message("return ${result}")
	SET(${result} ${dirlist})
endmacro()

function(set_relative_rpath tgt)
	if (UNIX)
		if (NOT APPLE)
			set_target_properties(${tgt} PROPERTIES BUILD_WITH_INSTALL_RPATH TRUE)
			set_target_properties(${tgt} PROPERTIES INSTALL_RPATH "$ORIGIN")
		endif()
	endif()
endfunction()
