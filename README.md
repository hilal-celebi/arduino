# ARDUINO ILE STATE CONTROL
## State Control Nedir?
Durum kontrolü, bir uygulamanın belirli bir andaki verilerini ve bu verilere bağlı işlemlerini yönetme sürecidir. Bu, hataları tespit etmeyi, veri tutarlılığını sağlamayı ve kullanıcı deneyimini iyileştirmeyi kolaylaştırır. 
Bu projedeki amacımız aracımız bellirli bir açıda sağa ve sola döndüğünde breadboardın en sağında ve en solunda olan ledlerimizin yanıp sönmesi,hızımız sıfırken ortadaki ledimizin sadece yanması ve hızımız sıfırın altındayken ortadaki ledimizin yanıp sönmesini sağlamak.

## Sağ ve Sol Ledin Yanıp Sönmesini Sağlayan Kod
Ledlerin belirli bir frekansta yanıp sönmesini sağlar.


```python
def blink_leds(self):
        self.current_time = time.time()

        if self.current_time - self.last_blink_time >= self.blink_interval:
            self.last_blink_time = self.current_time

            if self.left_turn_active:
                if self.left_turn_blink_state:
                    self.command_pub.publish("left_turn")
                else:
                    self.command_pub.publish("stop_left_turn")
                self.left_turn_blink_state = not self.left_turn_blink_state

            if self.right_turn_active:
                if self.right_turn_blink_state:
                    self.command_pub.publish("right_turn")
                else:
                    self.command_pub.publish("stop_right_turn")
                self.right_turn_blink_state = not self.right_turn_blink_state

```

## Sağa ve Sola Dönüşlerde Açısal Hızın Belirli Bir Eşiği Geçmesi Durumunda Ledlerin Yanması
Sağ ve sol dönüşlerde açısal hız belirli bir eşik değerinin geçip geçmemesini kontrol eden kod parçası aşağıdaki gibidir. Bu koda göre açısal hız solda 0.15 ve sağda -0.15 eşik değerini geçtiğinde ledler yanıp söncektir.

``` python
# Direksiyon kontrolü
        if self.steering_angle < -0.15:
            self.left_turn_active = False
            if not self.right_turn_active:
                self.right_turn_active = True
                self.right_turn_blink_state = True
                self.last_blink_time = time.time()

        elif self.steering_angle > 0.15:
            self.right_turn_active = False
            if not self.left_turn_active:
                self.left_turn_active = True
                self.left_turn_blink_state = True
                self.last_blink_time = time.time()
        else:
            self.left_turn_active = False
            self.right_turn_active = False
            self.command_pub.publish("stop_right_turn")
            self.command_pub.publish("stop_left_turn")

```


https://github.com/hilal-celebi/arduino/assets/153311166/3e34ecd8-b7c1-4b98-98c7-b18367a876fb 

https://github.com/hilal-celebi/arduino/assets/153311166/24710947-fa36-47ca-8278-dd6ee6fa0faf


## Hız Sıfırken Ledin Sürekli Yanması ve Hız Sıfırdan Küçükken Ledin Yanıp Sönmesini Sağlayan Kod

``` python
def stop_led(self):

        self.current_speed_time = time.time()

        if self.current_speed_time - self.last_speed_blink_time >= self.blink_interval:
            self.last_speed_blink_time = self.current_speed_time

        # Hız azalması kontrolü (hız negatifse)
        if self.current_speed < -0.01:
            if self.speed_blink_state:
                # rospy.loginfo("current_speed < 0 : Speed decrease LED ON")
                self.command_pub.publish("brake")
            else:
                self.command_pub.publish("stop_brake")
            self.speed_blink_state = not self.speed_blink_state

        elif self.current_speed > 0.01:  # Hız pozitifse
            # rospy.loginfo("current_speed > 0 : Speed decrease LED OFF")
            self.command_pub.publish("stop_brake")
            self.speed_blink_state = False
        else:  # Hız sıfırsa
            # rospy.loginfo("Speed is zero, LED ON")
            self.command_pub.publish("brake")

```

## Hızın Sıfıra Eşit veya Daha Küçük Olması Durumunun Eşiğini Belirleyen Kod
``` python

 if self.current_speed < -0.01:
            self.speed_blink_state = True
            self.last_speed_blink_time = time.time()
        elif self.current_speed > 0.01:
            self.speed_blink_state = False
        else:  # Hız sıfırsa
            self.speed_blink_state = False


```

https://github.com/hilal-celebi/arduino/assets/153311166/0263af21-b0bf-4206-907c-76c7d5ad85e5


https://github.com/hilal-celebi/arduino/assets/153311166/ee86a06b-4eda-430d-9537-b52c4483a226






## Aracın Hızının Azalması (İvmenin Negatif Olması) Durumunda Ledin Yanıp Sönmesi Sağlayan Kod
Aracımızın hızı pozitifken ivmenin negatif olması durumunda ortadaki kırmızı ledimiz yanıp sönecektir.
``` python

      def blink_leds_speed_acceleration(self):
        self.current_time = time.time()
        self.acceleration = (self.current_speed - self.last_speed) / (self.current_time - self.last_time)
    
        #  # İvme negatif kontrolü
        if self.acceleration < 0:
            rospy.loginfo("Acceleration decrease LED ON")
            self.command_pub.publish("brake")
    
        self.last_speed = self.current_speed
        self.last_time = self.current_time

```





https://github.com/hilal-celebi/arduino/assets/153311166/e22077ac-a223-41db-ae54-92442ca9eae9









