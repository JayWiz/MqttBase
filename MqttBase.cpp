#include "MqttBase.h"

MqttBase::MqttBase(const char* mqtt_server, const char* mqtt_client_name, uint16_t mqtt_port) {
  mqtt_server_ = mqtt_server;
  mqtt_port_ = mqtt_port;
  mqtt_client_name_ = mqtt_client_name;
}

MqttBase::~MqttBase() {}

void MqttBase::init(
    const char* ssid, const char* password, std::vector<std::shared_ptr<std::string>>& mqtt_topics,
    std::vector<std::function<void(const char*, const char*, int)>> logic_callbacks) {
  mqtt_topics_ = mqtt_topics;
  Serial.print("CONNECT TO WIFI");
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    debug_print(".");
  }
  debug_println("CONNECTED");

  pub_client_ = new PubSubClient();
  wifi_client_ = new WiFiClient();
  pub_client_->setClient(*wifi_client_);
  pub_client_->setServer(mqtt_server_, mqtt_port_);
  pub_client_->setCallback([this](char* topic, byte* payload, unsigned int length) {
    this->callback(topic, payload, length);
  });
  for (uint8_t i = 0; i < mqtt_topics_.size(); i++) {
    pub_client_->subscribe(mqtt_topics_[i]->c_str());
  }
  this->logic_callbacks_ = logic_callbacks;
}

void MqttBase::reconnect() {
  while (!pub_client_->connected()) {
    debug_print("Attempting MQTT connection...");
    // Attempt to connect
    if (pub_client_->connect(mqtt_client_name_)) {
      debug_println("connected");
      // Subscribe
      for (uint8_t i = 0; i < mqtt_topics_.size(); i++) {
        pub_client_->subscribe(mqtt_topics_[i]->c_str());
      }
    } else {
      debug_print("failed, rc=");
      debug_print(pub_client_->state());
      debug_println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}

void MqttBase::loop() {
  if (!pub_client_->connected()) {
    reconnect();
  }
  pub_client_->loop();
}

void MqttBase::callback(char* topic, byte* message, unsigned int length) {
  debug_print("Message arrived on topic: ");
  for (uint8_t i = 0; i < mqtt_topics_.size(); i++) {
    debug_print(mqtt_topics_[i]->c_str());
  }

  debug_print(". Message: ");
  String messageTemp;
  /*
    for (int i = 0; i < length; i++) {
      debug_print((char)message[i]);
      messageTemp += (char)message[i];
    }
    debug_println("");
  */
  deserializeJson(doc_, message);
  const char* method1 = doc_["method"];
  const char* state = doc_["state"];
  int daten = doc_["data"];

  debug_print("Methode: ");
  debug_println(method1);
  debug_print("State: ");
  debug_println(state);
  debug_print("Daten: ");
  debug_println(daten);

  for (uint8_t i = 0; i < logic_callbacks_.size(); i++) {
    if (strcmp(topic, mqtt_topics_[i]->c_str()) == 0) {
      logic_callbacks_[i](method1, state, daten);
    }
  }

  // logic_callback_(method1, state, daten);
}
/*
void MqttBase::publish(const char* methode, const char* state) {
  JSONencoder_["method"] = methode;
  JSONencoder_["state"] = state;
  JSONencoder_["data"] = 0;

  char JSONmessageBuffer[100];

  serializeJson(doc_, JSONmessageBuffer, 100);
  debug_print("send message");
  debug_println(JSONencoder_);

  pub_client_->publish(mqtt_topic_, JSONmessageBuffer);
}
*/
void MqttBase::debug_print(const char* str) {
#ifdef DEBUG
  Serial.print(str);
#endif
}

void MqttBase::debug_print(int i) {
#ifdef DEBUG
  Serial.print(i);
#endif
}

void MqttBase::debug_println(const char* str) {
#ifdef DEBUG
  Serial.println(str);
#endif
}

void MqttBase::debug_println(int i) {
#ifdef DEBUG
  Serial.println(i);
#endif
}