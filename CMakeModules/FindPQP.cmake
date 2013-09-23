# Copyright 2011, Olivier Stasse, JRL, CNRS/AIST
#
# This file is part of fast-replanning.
# fast-replanning is free software: you can redistribute it and/or
# modify it under the terms of the GNU Lesser General Public License
# as published by the Free Software Foundation, either version 3 of
# the License, or (at your option) any later version.
#
# fast-replanning is distributed in the hope that it will be useful, but
# WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
# General Lesser Public License for more details.  You should have
# received a copy of the GNU Lesser General Public License along with
# fast-replanning. If not, see <http://www.gnu.org/licenses/>.


# Test if robotpkg is here
FIND_PROGRAM(ROBOTPKG_INFO robotpkg_info)
# if it is the case
IF(ROBOTPKG_INFO)
  # add the prefix directory of robotpkg
  # to the directory to look for.
  EXECUTE_PROCESS(
      COMMAND ${ROBOTPKG_INFO} -Q PREFIX pqp
      WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
      RESULT_VARIABLE PQP_DESCRIBE_RESULT
      OUTPUT_VARIABLE PQP_DESCRIBE_OUTPUT
      ERROR_VARIABLE PQP_DESCRIBE_ERROR
      OUTPUT_STRIP_TRAILING_WHITESPACE
      )

ENDIF(ROBOTPKG_INFO)

FIND_PATH(PQP_INCLUDE_DIRS PQP.h
  /usr/include
  ${PQP_DESCRIBE_OUTPUT}/include/pqp
  )

IF(PQP_INCLUDE_DIRS)
  SET(PQP_FOUND 1)
  SET(PQP_LDFLAGS "-lPQP -L${PQP_DESCRIBE_OUTPUT}/lib")
  SET(PQP_INCLUDIRS_TMP "")
  FOREACH(PQP_INC_DIR ${PQP_INCLUDE_DIRS})
    LIST(APPEND PQP_INCLUDIRS_TMP ${PQP_INC_DIR})
    LIST(APPEND PQP_INCLUDIRS_TMP ${PQP_INC_DIR}/../)
  ENDFOREACH()
  SET(PQP_INCLUDE_DIRS ${PQP_INCLUDIRS_TMP})
ELSE(PQP_INCLUDE_DIRS)
  MESSAGE(ERROR "PQP not found")
ENDIF(PQP_INCLUDE_DIRS)

# For config.log
LIST(APPEND LOGGING_WATCHED_VARIABLES PQP_FOUND)
LIST(APPEND LOGGING_WATCHED_VARIABLES PQP_INCLUDE_DIRS)	
LIST(APPEND LOGGING_WATCHED_VARIABLES PQP_LDFLAGS)		


