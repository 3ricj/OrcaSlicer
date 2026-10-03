# FibreSeeker3 G-code contract tests.
#
# Pure-Python suite (no C++ target): fs_validate_fixtures.py drives the
# independent validator fs_gcode_validator.py over fixtures/*.gcode, each of
# which declares its expected findings as '# EXPECT: <LEVEL> <RULE>' comment
# lines. Any C++-independent Python 3 interpreter qualifies; when none is
# found the test is skipped at configure time rather than failing the build.

find_package(Python3 COMPONENTS Interpreter QUIET)

if (Python3_Interpreter_FOUND)
    set(FS_PYTHON "${Python3_EXECUTABLE}")
    set(FS_PYTHON_ARGS "")
else()
    # Windows launcher: 'py -3' resolves to the newest installed Python 3.
    find_program(FS_PY_LAUNCHER NAMES py)
    if (FS_PY_LAUNCHER)
        set(FS_PYTHON "${FS_PY_LAUNCHER}")
        set(FS_PYTHON_ARGS "-3")
    endif ()
endif ()

if (FS_PYTHON)
    add_test(NAME FibreSeekerFixtures
             COMMAND ${FS_PYTHON} ${FS_PYTHON_ARGS}
                     ${CMAKE_CURRENT_SOURCE_DIR}/fs_validate_fixtures.py
                     --dir ${CMAKE_CURRENT_SOURCE_DIR}/fixtures
             WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR})
    set_tests_properties(FibreSeekerFixtures PROPERTIES LABELS "fibreseeker")

    # P2 T1 plastic-only vertical slice: headless-slices the 20 mm cube with the
    # presets in presets/ through the real exporter, then validates the result
    # with the independent validator. Needs the built app and its resources/, so
    # it carries RequiresApp (like the cli/ suite); the script exits 3 (skip)
    # when the binary or its resources are missing. On Windows the OrcaSlicer
    # target is a DLL and the real app is the OrcaSlicer_app_gui executable.
    if (TARGET OrcaSlicer_app_gui)
        set(FS_APP_TARGET OrcaSlicer_app_gui)
    elseif (TARGET OrcaSlicer)
        set(FS_APP_TARGET OrcaSlicer)
    endif ()
    if (DEFINED FS_APP_TARGET)
        add_test(NAME FibreSeekerT1ExportSlice
                 COMMAND ${FS_PYTHON} ${FS_PYTHON_ARGS}
                         ${CMAKE_CURRENT_SOURCE_DIR}/fs_export_check.py
                         $<TARGET_FILE:${FS_APP_TARGET}>
                 WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR})
        set_tests_properties(FibreSeekerT1ExportSlice PROPERTIES
            LABELS "fibreseeker;RequiresApp"
            SKIP_RETURN_CODE 3
            TIMEOUT 900)

        # P5 composite fiber export: same driver in --fiber mode against the
        # fiber-enabled machine preset. The exported g-code must carry the
        # emitter lifecycle markers and validate with 0 ERRORs.
        add_test(NAME FibreSeekerFiberExport
                 COMMAND ${FS_PYTHON} ${FS_PYTHON_ARGS}
                         ${CMAKE_CURRENT_SOURCE_DIR}/fs_export_check.py
                         $<TARGET_FILE:${FS_APP_TARGET}>
                         --machine ${CMAKE_CURRENT_SOURCE_DIR}/presets/fs3_t1_fiber_machine.json
                         --fiber
                 WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR})
        set_tests_properties(FibreSeekerFiberExport PROPERTIES
            LABELS "fibreseeker;RequiresApp"
            SKIP_RETURN_CODE 3
            TIMEOUT 900)
    endif ()
else()
    message(STATUS "FibreSeeker3 tests skipped: no Python 3 interpreter found")
endif ()
