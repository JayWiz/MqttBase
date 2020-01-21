#include <ArduinoJson.h>
#include <PubSubClient.h>
#include <WiFi.h>
#include <vector>

#define DEBUG

class MqttBase {
 private:
  std::vector<std::shared_ptr<std::string>> mqtt_topics_;
  std::vector<std::function<void(const char*, const char*, int)>> logic_callbacks_;

  // const char* mqtt_topic_;
  const char* mqtt_server_;
  const char* mqtt_client_name_;
  int mqtt_port_ = 1883;
  StaticJsonDocument<300> doc_;
  JsonObject JSONencoder_ = doc_.to<JsonObject>();
  // std::unique_ptr<PubSubClient> client;
  PubSubClient* pub_client_;
  WiFiClient* wifi_client_;

  // std::function<void(const char*, const char*, int)> logic_callback_;

  void debug_print(const char* str);
  void debug_print(int i);
  void debug_println(const char* str);
  void debug_println(int i);

 public:
  MqttBase(const char* mqtt_server, const char* mqtt_client_name, uint16_t mqtt_port);
  ~MqttBase();

  void init(const char* ssid, const char* password,
            std::vector<std::shared_ptr<std::string>>& mqtt_topics,
            std::vector<std::function<void(const char*, const char*, int)>> logic_callbacks);
  void reconnect();
  virtual void callback(char* topic, byte* message, unsigned int length);
  virtual void publish(const char* topic, const char* methode, const char* state, bool retained);
  void loop();

  bool get_mqtt_status(return pub_client_->connected(););
};