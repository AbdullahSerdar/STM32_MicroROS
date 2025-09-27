[EN] V1.0: The system can receive IMU and GPS data from the Husky and GPS data from the drone, and it can move the Husky using /cmd_vel.
V1.1: The system receives IMU and GPS data from both the Husky and the drone, moves the Husky using cmd_vel, and converts the received IMU (quaternion) data into Euler angles.
V1.2: The system receives IMU and GPS data from both the drone and the Husky, publishes odometry data for both, and allows the drone to automatically move toward the Husky’s position when the user presses a button.
V1.3: The system receives IMU and GPS data from both the drone and the Husky, publishes odometry data for both, and allows the drone to automatically move toward the Husky when the user presses a button. Then, using the implemented vision algorithm, the drone aligns itself over the Husky, after which control returns to the Husky.
V1.4: Certain parts of V1.3 have been improved and the code has been simplified. No changes have been made to the core functionalities.

[TR] V1.0 : Sistemde Husky'nin İMU, GPS verisini ve Dronun GPS verisini alabiliyor ve /cmd_vel ile Husky'i harekete geçiriyor.

V1.1 : Sistemde Husky'nin ve Dronun İMU, GPS verisini alıyor, cmd_vel ile Husky'i harekete geçiriyor ve alınan imu(quaternion) verisini euler e çeviriyor.

V1.2 : Sistemde Drone ve Husky İMU,GPS verilerini alıyor ve hem husk hem de drone odyometri verilerini gönderiyor. Aynı zamanda kullanıcı butona bastığında drone otomatik olarak husky nin konumuna ilerliyor.

V1.3 : Sistemde Drone ve Husky İMU,GPS verilerini alıyor ve hem husk hem de drone odyometri verilerini gönderiyor. Aynı zamanda kullanıcı butona bastığında drone otomatik olarak husky nin konumuna ilerliyor sonra yazılan görüntü algoritması ile husky aracının üzerine yerleşiyor ve tekrar husky nin kontrolü başlıyor.

V1.4 : V1.3 ün belli kısımlarında iyileştirmeler yapılmış ve kod biraz daha sadeleştirilmiştir. Temel işlevleri bakımından bir değişiklik yapılmamıştır.
