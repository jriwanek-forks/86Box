typedef struct vserver_tftp_t vserver_tftp_t;
typedef void (*vserver_tftp_output_t)(void *priv, uint32_t destination, uint16_t source_port, uint16_t port,
                                      const uint8_t *data, int length);

vserver_tftp_t *vserver_tftp_create(vserver_tftp_output_t output, void *priv);
void            vserver_tftp_close(vserver_tftp_t *tftp);
bool            vserver_tftp_input(vserver_tftp_t *tftp, uint32_t source, uint16_t source_port,
                                   uint16_t destination_port, const uint8_t *data, int length);
void            vserver_tftp_tick(vserver_tftp_t *tftp);
