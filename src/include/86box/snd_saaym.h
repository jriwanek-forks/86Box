#ifndef SOUND_SAAYM_H
#define SOUND_SAAYM_H

typedef struct saaym_t {
    int     addr;

    uint8_t latched_data;

    cms_t   cms;

} saaym_t;

#endif /*SOUND_SAAYM_H*/
