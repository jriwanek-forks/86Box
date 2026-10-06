if(BUILD_TESTING)
    find_package(ZLIB REQUIRED)
    function(add_net_modem_test target test_source)
        add_executable(${target} ${test_source} ${ARGN})
        target_include_directories(${target} PRIVATE
            ${PROJECT_SOURCE_DIR}/src/include
            ${PROJECT_BINARY_DIR}/src/include
        )
        target_link_libraries(${target} PRIVATE GTest::gtest_main)
        gtest_discover_tests(${target})
    endfunction()

    add_executable(net_modem_crypto_tests
        net_modem_crypto_test.cpp
        ../../src/network/net_modem_crypto.c
    )
    target_include_directories(net_modem_crypto_tests PRIVATE
        ${PROJECT_SOURCE_DIR}/src/include
        ${PROJECT_BINARY_DIR}/src/include
    )
    target_link_libraries(net_modem_crypto_tests PRIVATE GTest::gtest_main)
    include(GoogleTest)
    gtest_discover_tests(net_modem_crypto_tests)

    add_executable(net_modem_eap_tests
        net_modem_eap_test.cpp
        ../../src/network/net_modem_eap.c
        ../../src/network/net_modem_crypto.c
    )
    target_include_directories(net_modem_eap_tests PRIVATE
        ${PROJECT_SOURCE_DIR}/src/include
        ${PROJECT_BINARY_DIR}/src/include
    )
    target_link_libraries(net_modem_eap_tests PRIVATE GTest::gtest_main)
    gtest_discover_tests(net_modem_eap_tests)

    add_net_modem_test(net_modem_ppp_tests net_modem_ppp_test.cpp
        ../../src/network/net_modem_ppp.c
        ../../src/network/net_modem_pap.c
        ../../src/network/net_modem_chap.c
        ../../src/network/net_modem_ccp.c
        ../../src/network/net_modem_ccp_codec.c
        ../../src/network/net_modem_eap.c
        ../../src/network/net_modem_ipcp.c
        ../../src/network/net_modem_cslip.c
        ../../src/network/net_modem_mppe.c
        ../../src/network/net_modem_crypto.c
        ../../src/network/net_modem_mppp.c
    )
    target_link_libraries(net_modem_ppp_tests PRIVATE ZLIB::ZLIB)
    if(WIN32)
        target_link_libraries(net_modem_ppp_tests PRIVATE bcrypt)
    endif()

    add_net_modem_test(net_modem_mppp_tests net_modem_mppp_test.cpp
        ../../src/network/net_modem_mppp.c
    )

    add_net_modem_test(net_modem_pap_tests net_modem_pap_test.cpp
        ../../src/network/net_modem_pap.c
    )
    add_net_modem_test(net_modem_chap_tests net_modem_chap_test.cpp
        ../../src/network/net_modem_chap.c
        ../../src/network/net_modem_mppe.c
        ../../src/network/net_modem_crypto.c
    )
    add_net_modem_test(net_modem_mppe_tests net_modem_mppe_test.cpp
        ../../src/network/net_modem_mppe.c
        ../../src/network/net_modem_crypto.c
    )
    add_net_modem_test(net_modem_ccp_tests net_modem_ccp_test.cpp
        ../../src/network/net_modem_ccp.c
        ../../src/network/net_modem_ccp_codec.c
        ../../src/network/net_modem_mppe.c
        ../../src/network/net_modem_crypto.c
    )
    target_link_libraries(net_modem_ccp_tests PRIVATE ZLIB::ZLIB)
    add_net_modem_test(net_modem_ipcp_tests net_modem_ipcp_test.cpp
        ../../src/network/net_modem_ipcp.c
    )
    add_net_modem_test(net_modem_cslip_tests net_modem_cslip_test.cpp
        ../../src/network/net_modem_cslip.c
    )
    add_net_modem_test(net_modem_slip_tests net_modem_slip_test.cpp
        ../../src/network/net_modem_slip.c
    )
    add_net_modem_test(net_modem_slip_auth_tests net_modem_slip_auth_test.cpp
        ../../src/network/net_modem_slip_auth.c
    )
endif()