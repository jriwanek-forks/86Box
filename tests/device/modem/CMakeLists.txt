if(BUILD_TESTING)
    find_package(ZLIB REQUIRED)
    function(add_modem_test target test_source)
        add_executable(${target} ${test_source} ${ARGN})
        target_include_directories(${target} PRIVATE
            ${PROJECT_SOURCE_DIR}/src/include
            ${PROJECT_BINARY_DIR}/src/include
        )
        target_link_libraries(${target} PRIVATE GTest::gtest_main)
        gtest_discover_tests(${target})
    endfunction()

    add_executable(modem_crypto_tests
        modem_crypto_test.cpp
        ../../../src/device/modem/modem_crypto.c
    )
    target_include_directories(modem_crypto_tests PRIVATE
        ${PROJECT_SOURCE_DIR}/src/include
        ${PROJECT_BINARY_DIR}/src/include
    )
    target_link_libraries(modem_crypto_tests PRIVATE GTest::gtest_main)
    include(GoogleTest)
    gtest_discover_tests(modem_crypto_tests)

    add_executable(modem_eap_tests
        modem_eap_test.cpp
        ../../../src/device/modem/modem_eap.c
        ../../../src/device/modem/modem_crypto.c
    )
    target_include_directories(modem_eap_tests PRIVATE
        ${PROJECT_SOURCE_DIR}/src/include
        ${PROJECT_BINARY_DIR}/src/include
    )
    target_link_libraries(modem_eap_tests PRIVATE GTest::gtest_main)
    gtest_discover_tests(modem_eap_tests)

    add_modem_test(modem_ppp_tests modem_ppp_test.cpp
        ../../../src/device/modem/modem_ppp.c
        ../../../src/device/modem/modem_pap.c
        ../../../src/device/modem/modem_chap.c
        ../../../src/device/modem/modem_ccp.c
        ../../../src/device/modem/modem_ccp_codec.c
        ../../../src/device/modem/modem_eap.c
        ../../../src/device/modem/modem_ipcp.c
        ../../../src/device/modem/modem_cslip.c
        ../../../src/device/modem/modem_mppe.c
        ../../../src/device/modem/modem_crypto.c
        ../../../src/device/modem/modem_mppp.c
    )
    target_link_libraries(modem_ppp_tests PRIVATE ZLIB::ZLIB)
    if(WIN32)
        target_link_libraries(modem_ppp_tests PRIVATE bcrypt)
    endif()

    add_modem_test(modem_mppp_tests modem_mppp_test.cpp
        ../../../src/device/modem/modem_mppp.c
    )

    add_modem_test(modem_pap_tests modem_pap_test.cpp
        ../../../src/device/modem/modem_pap.c
        ../../../src/device/modem/modem_crypto.c
    )
    add_modem_test(modem_chap_tests modem_chap_test.cpp
        ../../../src/device/modem/modem_chap.c
        ../../../src/device/modem/modem_mppe.c
        ../../../src/device/modem/modem_crypto.c
    )
    add_modem_test(modem_mppe_tests modem_mppe_test.cpp
        ../../../src/device/modem/modem_mppe.c
        ../../../src/device/modem/modem_crypto.c
    )
    add_modem_test(modem_ccp_tests modem_ccp_test.cpp
        ../../../src/device/modem/modem_ccp.c
        ../../../src/device/modem/modem_ccp_codec.c
        ../../../src/device/modem/modem_mppe.c
        ../../../src/device/modem/modem_crypto.c
    )
    target_link_libraries(modem_ccp_tests PRIVATE ZLIB::ZLIB)
    add_modem_test(modem_ipcp_tests modem_ipcp_test.cpp
        ../../../src/device/modem/modem_ipcp.c
    )
    add_modem_test(modem_cslip_tests modem_cslip_test.cpp
        ../../../src/device/modem/modem_cslip.c
    )
    add_modem_test(modem_slip_tests modem_slip_test.cpp
        ../../../src/device/modem/modem_slip.c
    )
    add_modem_test(modem_slip_auth_tests modem_slip_auth_test.cpp
        ../../../src/device/modem/modem_slip_auth.c
        ../../../src/device/modem/modem_crypto.c
    )

    add_modem_test(modem_voice_tests modem_voice_test.cpp
        ../../../src/device/modem/modem_voice.c
    )
    if(NOT MSVC)
        target_link_libraries(modem_voice_tests PRIVATE m)
    endif()
endif()
