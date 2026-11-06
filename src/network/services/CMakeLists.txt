target_sources(net PRIVATE
    net_browser.c
    net_ipx.c
    net_nbf.c
    net_nbns.c
    net_smb.c
    net_smb_fs.c
    net_smb_rap.c
    net_vserver.c
    net_vserver_eth.c
)

if(ENABLE_VSERVER_FTP)
    target_sources(net PRIVATE net_ftp.c)
endif()

if(ENABLE_VSERVER_TFTP)
    target_sources(net PRIVATE net_tftp.c)
endif()
