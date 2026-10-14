/*
 * 86Box    A hypervisor and IBM PC system emulator that specializes in
 *          running old operating systems and software designed for IBM
 *          PC systems and compatibles from 1981 through fairly recent
 *          system designs based on the PCI bus.
 *
 *          This file is part of the 86Box distribution.
 *
 *          Define the various paper sizes for printers.
 *
 * Authors: Jasmine Iwanek, <jasmine@iwanek.co.uk>
 *
 *          Copyright 2025-2026 Jasmine Iwanek
 */
#ifndef EMU_PRT_PAPERSIZES_H
#define EMU_PRT_PAPERSIZES_H

/* Standard U.S. Letter */
#define LETTER_PAGE_WIDTH   8.5
#define LETTER_PAGE_HEIGHT  11.0

/* Standard U.S. Legal */
#define LEGAL_PAGE_WIDTH   8.5
#define LEGAL_PAGE_HEIGHT  14.0

/* Standard U.S. Ledger */
#define LEDGER_PAGE_WIDTH   11.0
#define LEDGER_PAGE_HEIGHT  17.0

/* Standard A0 */
#define A0_PAGE_WIDTH   33.110236
#define A0_PAGE_HEIGHT  46.811023

/* Standard A1 */
#define A1_PAGE_WIDTH   23.385826
#define A1_PAGE_HEIGHT  33.110236

/* Standard A2 */
#define A2_PAGE_WIDTH   16.535433
#define A2_PAGE_HEIGHT  23.385826

/* Standard A3 */
#define A3_PAGE_WIDTH   11.692913
#define A3_PAGE_HEIGHT  16.535433

/* Standard A4 */
#define A4_PAGE_WIDTH   8.267716
#define A4_PAGE_HEIGHT  11.692913

/* Standard A5 */
#define A5_PAGE_WIDTH   5.826772
#define A5_PAGE_HEIGHT  8.267716

/* Standard A6 */
#define A6_PAGE_WIDTH   4.133858
#define A6_PAGE_HEIGHT  5.826772

/* Standard A7 */
#define A7_PAGE_WIDTH   2.913386
#define A7_PAGE_HEIGHT  4.133858

/* Standard A8 */
#define A8_PAGE_WIDTH   2.047244
#define A8_PAGE_HEIGHT  2.913386

/* Standard A9 */
#define A9_PAGE_WIDTH   1.456693
#define A9_PAGE_HEIGHT  2.047244

/* Standard A10 */
#define A10_PAGE_WIDTH  1.023622
#define A10_PAGE_HEIGHT 1.456693

/* Standard B0 */
#define B0_PAGE_WIDTH   39.370079
#define B0_PAGE_HEIGHT  55.669291

/* Standard B1 */
#define B1_PAGE_WIDTH   27.834646
#define B1_PAGE_HEIGHT  39.370079

/* Standard B2 */
#define B2_PAGE_WIDTH   19.685039
#define B2_PAGE_HEIGHT  27.834646

/* Standard B3 */
#define B3_PAGE_WIDTH   13.897638
#define B3_PAGE_HEIGHT  19.685039

/* Standard B4 */
#define B4_PAGE_WIDTH   9.8425197
#define B4_PAGE_HEIGHT  13.897637

/* Standard B5 */
#define B5_PAGE_WIDTH   6.929134
#define B5_PAGE_HEIGHT  9.842520

/* Standard B6 */
#define B6_PAGE_WIDTH   4.921260
#define B6_PAGE_HEIGHT  6.929134

/* Standard B7 */
#define B7_PAGE_WIDTH   3.464567
#define B7_PAGE_HEIGHT  4.921260

/* Standard B8 */
#define B8_PAGE_WIDTH   2.440945
#define B8_PAGE_HEIGHT  3.464567

/* Standard B9 */
#define B9_PAGE_WIDTH   1.732283
#define B9_PAGE_HEIGHT  2.440945

/* Standard B10 */
#define B10_PAGE_WIDTH  1.220472
#define B10_PAGE_HEIGHT 1.732283

/* Common U.S. business paper sizes */
#define EXECUTIVE_PAGE_WIDTH   7.25
#define EXECUTIVE_PAGE_HEIGHT  10.5
#define STATEMENT_PAGE_WIDTH   5.5
#define STATEMENT_PAGE_HEIGHT  8.5
#define FOLIO_PAGE_WIDTH       8.5
#define FOLIO_PAGE_HEIGHT      13.0
#define TEN_BY_FOURTEEN_PAGE_WIDTH  10.0
#define TEN_BY_FOURTEEN_PAGE_HEIGHT 14.0
#define ANSI_C_PAGE_WIDTH      17.0
#define ANSI_C_PAGE_HEIGHT     22.0
#define ANSI_D_PAGE_WIDTH      22.0
#define ANSI_D_PAGE_HEIGHT     34.0
#define ANSI_E_PAGE_WIDTH      34.0
#define ANSI_E_PAGE_HEIGHT     44.0

/* Common envelope sizes */
#define ENVELOPE_DL_PAGE_WIDTH      4.330709
#define ENVELOPE_DL_PAGE_HEIGHT     8.661417
#define ENVELOPE_C5_PAGE_WIDTH      6.377953
#define ENVELOPE_C5_PAGE_HEIGHT     9.015748
#define ENVELOPE_NO10_PAGE_WIDTH    4.125
#define ENVELOPE_NO10_PAGE_HEIGHT   9.5
#define ENVELOPE_MONARCH_PAGE_WIDTH 3.875
#define ENVELOPE_MONARCH_PAGE_HEIGHT 7.5

#endif /* EMU_PRT_PAPERSIZES_H */
