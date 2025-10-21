/*
 * 86Box    A hypervisor and IBM PC system emulator that specializes in
 *          running old operating systems and software designed for IBM
 *          PC systems and compatibles from 1981 through fairly recent
 *          system designs based on the PCI bus.
 *
 *          This file is part of the 86Box distribution.
 *
 *          Header file for Linux CD-ROM change notification for QT
 *
 * Authors: GitHub Copilot
 *
 *          Copyright 2025 86Box contributors.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 */
#ifndef QT_LINUXCDROMNOTIFY_HPP
#define QT_LINUXCDROMNOTIFY_HPP

#include <QObject>
#include <QMainWindow>
#include <QSocketNotifier>
#include <QTimer>

#include <memory>
#include <sys/inotify.h>

#include "qt_mainwindow.hpp"

class LinuxCDROMNotify : public QObject {
    Q_OBJECT

public:
    static std::unique_ptr<LinuxCDROMNotify> Register(MainWindow *window);
    
    ~LinuxCDROMNotify();

private slots:
    void onInotifyEvent();

private:
    MainWindow      *window;
    int              inotify_fd;
    QSocketNotifier *notifier;
    QTimer          *periodic_timer;
    
    struct CDROMDevice {
        QString  path;
        int      watch_fd;
        time_t   last_check;
        uint32_t last_capacity;
        dev_t    last_device_id;
        int      cdrom_id;
    };
    
    QList<CDROMDevice> monitored_devices;
    
    LinuxCDROMNotify(MainWindow *window);
    
    void setupCDROMMonitoring();    
    void processCDROMChange(const QString &path, int cdrom_id);
    bool addCDROMDevice(const QString &path, int cdrom_id);    
};
#endif /*QT_LINUXCDROMNOTIFY_HPP*/
