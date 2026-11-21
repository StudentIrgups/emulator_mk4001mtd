sudo apt update
sudo apt install -y linux-headers-$(uname -r) build-essential git device-tree-compiler \
                    ti-pru-software-v6.0 ti-pru-cgt-v2.3

sudo ln -sf /usr/lib/ti/pru-software-support-package-v6.0 /usr/lib/ti/pru-software-support-package

cd /opt
sudo git clone https://github.com/abhishek-kakkar/BeagleLogic.git
sudo chown -R $USER:$USER /opt/BeagleLogic

cd /opt/BeagleLogic/kernel

# 1. Убираем несуществующий заголовок pruss_intc.h
sed -i 's|#include <linux/pruss_intc.h>|/* #include <linux/pruss_intc.h> */|' beaglelogic.c

# 2. Заменяем вызов прерывания на стандартный для ядра 5.10
sed -i 's/pruss_intc_trigger(bldev->to_bl_irq);/irq_set_irqchip_state(bldev->to_bl_irq, IRQCHIP_STATE_PENDING, true);/' beaglelogic.c

# 3. Добавляем 3-й аргумент NULL в pru_rproc_get
sed -i 's/pru_rproc_get(node, PRUSS_PRU0)/pru_rproc_get(node, PRUSS_PRU0, NULL)/' beaglelogic.c
sed -i 's/pru_rproc_get(node, PRUSS_PRU1)/pru_rproc_get(node, PRUSS_PRU1, NULL)/' beaglelogic.c

# 4. Задаём 32-битную маску DMA для miscdevice
sed -i '/ret = misc_register(&bldev->miscdev);/a \	if (!ret) dma_coerce_mask_and_coherent(bldev->miscdev.this_device, DMA_BIT_MASK(32));' beaglelogic.c

# 5. Сборка и установка модуля
make -C /lib/modules/$(uname -r)/build M=$(pwd) modules
sudo make -C /lib/modules/$(uname -r)/build M=$(pwd) modules_install
sudo depmod -a

python3 - << 'EOF'
path = "/opt/BeagleLogic/firmware/beaglelogic-pru0.c"
with open(path, "r") as f:
    code = f.read()

patch = """CT_CFG.SYSCFG_bit.STANDBY_INIT = 0;

    /* Настройка маршрутизации INTC (замена resource_table_0.h для ядра 5.10) */
    CT_INTC.CMR4_bit.CH_MAP_16 = 2; /* SYSEV_PRU0_TO_ARM (16) -> Channel 2 */
    CT_INTC.CMR4_bit.CH_MAP_17 = 0; /* SYSEV_ARM_TO_PRU0 (17) -> Channel 0 */
    CT_INTC.CMR5_bit.CH_MAP_20 = 0; /* SYSEV_PRU1_TO_PRU0 (20) -> Channel 0 */
    CT_INTC.CMR5_bit.CH_MAP_22 = 4; /* SYSEV_PRU0_TO_ARM_A (22) -> Channel 4 */
    CT_INTC.CMR5_bit.CH_MAP_23 = 1; /* SYSEV_ARM_TO_PRU0_A (23) -> Channel 1 */
    CT_INTC.CMR6_bit.CH_MAP_24 = 5; /* SYSEV_PRU0_TO_ARM_B (24) -> Channel 5 */

    CT_INTC.HMR0_bit.HINT_MAP_0 = 0; /* Channel 0 -> Host 0 (R31 бит 30 на PRU0) */
    CT_INTC.HMR0_bit.HINT_MAP_1 = 1; /* Channel 1 -> Host 1 */
    CT_INTC.HMR0_bit.HINT_MAP_2 = 2; /* Channel 2 -> Host 2 */
    CT_INTC.HMR1_bit.HINT_MAP_4 = 4; /* Channel 4 -> Host 4 */
    CT_INTC.HMR1_bit.HINT_MAP_5 = 5; /* Channel 5 -> Host 5 */

    CT_INTC.EISR = 16;
    CT_INTC.EISR = 17;
    CT_INTC.EISR = 20;
    CT_INTC.EISR = 22;
    CT_INTC.EISR = 23;
    CT_INTC.EISR = 24;

    CT_INTC.HIEISR = 0;
    CT_INTC.HIEISR = 1;
    CT_INTC.HIEISR = 2;
    CT_INTC.HIEISR = 4;
    CT_INTC.HIEISR = 5;
    CT_INTC.GER = 1;"""

assert "CT_CFG.SYSCFG_bit.STANDBY_INIT = 0;" in code, "Не найдена точка вставки!"
code = code.replace("CT_CFG.SYSCFG_bit.STANDBY_INIT = 0;", patch, 1)

with open(path, "w") as f:
    f.write(code)
print("Патч успешно применён.")
EOF


cd /opt/BeagleLogic/firmware
cp resource_table_1.h resource_table_0.h
make clean && make

sudo cp release/beaglelogic-pru0.out /lib/firmware/beaglelogic-pru0-fw
sudo cp release/beaglelogic-pru1.out /lib/firmware/beaglelogic-pru1-fw


cat << 'EOF' > /opt/BeagleLogic/kernel/beaglelogic-00A0.dts
/dts-v1/;
/plugin/;

/ {
        compatible = "ti,beaglebone", "ti,beaglebone-black", "ti,beaglebone-green";
        part-number = "BEAGLELOGIC";
        version = "00A0";

        fragment@0 {
                target-path = "/";
                __overlay__ {
                        pru-beaglelogic {
                                compatible = "beaglelogic,beaglelogic";
                                samplerate = <50000000>;
                                sampleunit = <1>;
                                triggerflags = <0>;

                                ti,prus = <&pru0>, <&pru1>;
                                firmware-name = "beaglelogic-pru0-fw", "beaglelogic-pru1-fw";
                                interrupt-parent = <&pruss_intc>;
                                interrupts = <22 4 4>, <23 1 1>, <24 5 5>,
                                             <16 2 2>, <17 0 0>, <20 0 0>;
                                interrupt-names = "from_bl_1", "to_bl", "from_bl_2",
                                                  "pru0_to_arm", "arm_to_pru0", "pru1_to_pru0";
                        };
                };
        };
};
EOF

dtc -O dtb -o /tmp/beaglelogic-00A0.dtbo -b 0 -@ /opt/BeagleLogic/kernel/beaglelogic-00A0.dts
sudo cp /tmp/beaglelogic-00A0.dtbo /lib/firmware/beaglelogic-00A0.dtbo


# 1. Освобождаем пины P8_27..P8_46 от HDMI
sudo sed -i 's/^#disable_uboot_overlay_video=1/disable_uboot_overlay_video=1/' /boot/uEnv.txt

# 2. Подключаем оверлей в слот addr4
sudo sed -i 's|^#uboot_overlay_addr4=.*|uboot_overlay_addr4=/lib/firmware/beaglelogic-00A0.dtbo|' /boot/uEnv.txt

# 3. Проверяем активные строки оверлеев перед перезагрузкой
grep -E "^(enable_uboot_overlays|disable_uboot_overlay_video|uboot_overlay_addr4)=" /boot/uEnv.txt

# 4. Добавляем модуль в автозагрузку
echo "beaglelogic" | sudo tee /etc/modules-load.d/beaglelogic.conf


# 5. Перезагружаем плату
sudo reboot


sudo tee /etc/systemd/system/beaglelogic-setup.service << 'EOF'
[Unit]
Description=Configure BeagleLogic pins and DMA buffer
After=multi-user.target

[Service]
Type=oneshot
ExecStart=/bin/bash -c 'for p in P8_45 P8_46 P8_43 P8_44 P8_41 P8_42 P8_39 P8_40 P8_27 P8_29 P8_28 P8_30; do config-pin $p pruin; done; echo 134217728 > /sys/devices/virtual/misc/beaglelogic/memalloc'
RemainAfterExit=yes

[Install]
WantedBy=multi-user.target
EOF

sudo systemctl daemon-reload
sudo systemctl enable beaglelogic-setup.service


python3 - << 'EOF'
path = "/opt/BeagleLogic/kernel/beaglelogic.c"
with open(path, "r") as f:
    code = f.read()

old_func = """unsigned int beaglelogic_f_poll(struct file *filp,
\t\tstruct poll_table_struct *tbl)
{
\tstruct logic_buffer_reader *reader = filp->private_data;
\tstruct beaglelogicdev *bldev = reader->bldev;
\tstruct logic_buffer *buf;

\t/* Raise an error if polled without starting the LA first */
\tif (reader->buf == NULL && bldev->state != STATE_BL_RUNNING)
\t\treturn -ENOEXEC;

\tbuf = reader->buf;
\tif (buf->state == STATE_BL_BUF_UNMAPPED)
\t\treturn (POLLIN | POLLRDNORM);

\tpoll_wait(filp, &bldev->wait, tbl);

\treturn 0;
}"""

new_func = """unsigned int beaglelogic_f_poll(struct file *filp,
\t\tstruct poll_table_struct *tbl)
{
\tstruct logic_buffer_reader *reader = filp->private_data;
\tstruct beaglelogicdev *bldev;
\tstruct logic_buffer *buf;

\tif (!reader || !reader->bldev)
\t\treturn EPOLLERR;

\tbldev = reader->bldev;
\tpoll_wait(filp, &bldev->wait, tbl);

\tbuf = reader->buf;
\tif (!buf)
\t\treturn 0;

\tif (buf->state == STATE_BL_BUF_UNMAPPED)
\t\treturn (POLLIN | POLLRDNORM);

\treturn 0;
}"""

assert old_func in code, "Не найден точный блок beaglelogic_f_poll!"
code = code.replace(old_func, new_func, 1)

with open(path, "w") as f:
    f.write(code)
print("Функция beaglelogic_f_poll успешно обновлена.")
EOF
