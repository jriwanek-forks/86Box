#ifndef QT_SETTINGSOTHERREMOVABLE_HPP
#define QT_SETTINGSOTHERREMOVABLE_HPP

#include <QWidget>

namespace Ui {
class SettingsOtherRemovable;
}

class SettingsOtherRemovable : public QWidget {
    Q_OBJECT

public:
    explicit SettingsOtherRemovable(QWidget *parent = nullptr);
    ~SettingsOtherRemovable();

    void save();

#if 0
private slots:
    void on_checkBoxZIP250_stateChanged(int arg1);

private slots:
    void on_comboBoxZIPChannel_activated(int index);

private slots:
    void on_comboBoxZIPBus_activated(int index);

private slots:
    void on_comboBoxZIPBus_currentIndexChanged(int index);
#endif

private slots:
    void on_comboBoxMOType_activated(int index);

private slots:
    void on_comboBoxMOChannel_activated(int index);

private slots:
    void on_comboBoxMOBus_activated(int index);

private slots:
    void on_comboBoxMOBus_currentIndexChanged(int index);

private slots:
    void onMORowChanged(const QModelIndex &current);
#if 0
    void onZIPRowChanged(const QModelIndex &current);
#endif

private:
    Ui::SettingsOtherRemovable *ui;
};

#endif // QT_SETTINGSOTHERREMOVABLE_HPP
