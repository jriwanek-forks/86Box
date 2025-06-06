#ifndef QT_SETTINGSDISPLAY_HPP
#define QT_SETTINGSDISPLAY_HPP

#include <QWidget>

#define VIDEOCARD_MAX 2

namespace Ui {
class SettingsDisplay;
}

class SettingsDisplay : public QWidget {
    Q_OBJECT

public:
    explicit SettingsDisplay(QWidget *parent = nullptr);
    ~SettingsDisplay();

    void save();

public slots:
    void onCurrentMachineChanged(int machineId);

private slots:
    void on_comboBoxVideo_currentIndexChanged(int index);
    void on_pushButtonConfigureVideo_clicked();

    void on_comboBoxVideoSecondary_currentIndexChanged(int index);
    void on_pushButtonConfigureVideoSecondary_clicked();

    void on_checkBoxVoodoo_stateChanged(int state);
    void on_pushButtonConfigureVoodoo_clicked();

    void on_checkBox8514_stateChanged(int state);
    void on_pushButtonConfigure8514_clicked();

    void on_checkBoxXga_stateChanged(int state);
    void on_pushButtonConfigureXga_clicked();

    void on_checkBoxDa2_stateChanged(int state);
    void on_pushButtonConfigureDa2_clicked();

private:
    Ui::SettingsDisplay *ui;
    int                  machineId    = 0;
    int                  videoCard[VIDEOCARD_MAX] = { 0, 0 };
};

#endif // QT_SETTINGSDISPLAY_HPP
