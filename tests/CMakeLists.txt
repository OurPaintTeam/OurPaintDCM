# Рекурсивно ищем все .cpp файлы в tests/ и подпапках
file(GLOB_RECURSE TEST_SOURCES CONFIGURE_DEPENDS *.cpp)
# Исключаем перформанс-тест drag из автоподхвата
list(FILTER TEST_SOURCES EXCLUDE REGEX ".*/DragOptimizersBenchmarkGTEST\\.cpp$")

find_package(Threads REQUIRED)

foreach(TEST_SRC ${TEST_SOURCES})
    # Имя теста без пути и расширения
    get_filename_component(TEST_NAME ${TEST_SRC} NAME_WE)

    add_executable(${TEST_NAME} ${TEST_SRC})

    # Линкуем с библиотекой и GTest
    target_link_libraries(${TEST_NAME} PRIVATE OurPaintDCM gtest gtest_main Threads::Threads)

    # Добавляем тест в CTest
    add_test(NAME ${TEST_NAME} COMMAND ${TEST_NAME})
    if(TEST_NAME MATCHES "Benchmark$")
        set_tests_properties(${TEST_NAME} PROPERTIES LABELS "perf")
    else()
        set_tests_properties(${TEST_NAME} PROPERTIES LABELS "unit;dcm" TIMEOUT 120)
    endif()
endforeach()
