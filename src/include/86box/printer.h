/*
 * VARCem   Virtual ARchaeological Computer EMulator.
 *          An emulator of (mostly) x86-based PC systems and devices,
 *          using the ISA,EISA,VLB,MCA  and PCI system buses, roughly
 *          spanning the era between 1981 and 1995.
 *
 *          Definitions for the printers module.
 *
 * Authors: Fred N. van Kempen, <decwiz@yahoo.com>
 *
 *          Copyright 2018 Fred N. van Kempen.
 *
 *          Redistribution and  use  in source  and binary forms, with
 *          or  without modification, are permitted  provided that the
 *          following conditions are met:
 *
 *          1. Redistributions of  source  code must retain the entire
 *             above notice, this list of conditions and the following
 *             disclaimer.
 *
 *          2. Redistributions in binary form must reproduce the above
 *             copyright  notice,  this list  of  conditions  and  the
 *             following disclaimer in  the documentation and/or other
 *             materials provided with the distribution.
 *
 *          3. Neither the  name of the copyright holder nor the names
 *             of  its  contributors may be used to endorse or promote
 *             products  derived from  this  software without specific
 *             prior written permission.
 *
 * THIS SOFTWARE  IS  PROVIDED BY THE  COPYRIGHT  HOLDERS AND CONTRIBUTORS
 * "AS IS" AND  ANY EXPRESS  OR  IMPLIED  WARRANTIES,  INCLUDING, BUT  NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
 * PARTICULAR PURPOSE  ARE  DISCLAIMED. IN  NO  EVENT  SHALL THE COPYRIGHT
 * HOLDER OR  CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL,  EXEMPLARY,  OR  CONSEQUENTIAL  DAMAGES  (INCLUDING,  BUT  NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE  GOODS OR SERVICES;  LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED  AND ON  ANY
 * THEORY OF  LIABILITY, WHETHER IN  CONTRACT, STRICT  LIABILITY, OR  TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING  IN ANY  WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
#ifndef PRINTER_H
#define PRINTER_H

#define FONT_FILE_DOTMATRIX        "dotmatrix.otf"
#define FONT_FILE_DOTMATRIX_ITALIC "dotmatrix_italic.otf"

#define FONT_FILE_ROMAN     "roman.ttf"
#define FONT_FILE_SANSSERIF "sansserif.ttf"
#define FONT_FILE_COURIER   "courier.ttf"
#define FONT_FILE_SCRIPT    "script.ttf"
#define FONT_FILE_OCRA      "ocra.ttf"
#define FONT_FILE_OCRB      "ocra.ttf"

extern void
select_codepage(uint16_t code, uint16_t *curmap);

/* FreeType handle, shared between the printers which render text. */
#ifdef FT_FREETYPE_H
extern FT_Library ft_lib;
#endif

/* HP DeskJet 500 series PCL 3 page engine. */
extern void *pcl3_open(double width, double height, int dpi);
extern void  pcl3_write(void *priv, uint8_t ch);
extern void  pcl3_flush(void *priv);
extern void  pcl3_reset(void *priv);
extern void  pcl3_close(void *priv);
extern void  pcl3_load_cartridge(void *priv, const char *fn);

#endif /*PRINTER_H*/
