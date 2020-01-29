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
  int counter = 0;
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    debug_print(".");
    if (counter > 10) {
      WiFi.disconnect();
      delay(1000);
      WiFi.begin(ssid, password);
      counter = 0;
    }
    counter++;
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
  debug_print(topic);

  debug_print(". Message: ");

  // just parse message if it contains at least 6 elements
  if (length > 6) {
    DeserializationError err = deserializeJson(doc_, message);
    switch (err.code()) {
      case DeserializationError::Ok:
        debug_println("Deserialization succeeded");
        break;
      case DeserializationError::InvalidInput:
        debug_println("Invalid input!");
        break;
      case DeserializationError::NoMemory:
        debug_println("Not enough memory");
        break;
      default:
        debug_println("Deserialization failed");
        break;
    }
    if (err.code() == DeserializationError::Ok) {
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
    }
  } else {
    debug_println("Received message too short! Not parsed.");
  }
}

void MqttBase::publish(const char* topic, const char* methode, const char* state, int data,
                       bool retained) {
  JSONencoder_["method"] = methode;
  JSONencoder_["state"] = state;
  JSONencoder_["data"] = data;

  char JSONmessageBuffer[100];

  serializeJson(doc_, JSONmessageBuffer, 100);
  debug_print("send message");
  debug_println(JSONencoder_);

  pub_client_->publish(topic, JSONmessageBuffer, retained);
}

void MqttBase::publish(const char* topic, const char* methode, const char* state, bool retained) {
  JSONencoder_["method"] = methode;
  JSONencoder_["state"] = state;
  JSONencoder_["data"] = 0;

  char JSONmessageBuffer[100];

  serializeJson(doc_, JSONmessageBuffer, 100);
  debug_print("send message");
  debug_println(JSONencoder_);

  pub_client_->publish(topic, JSONmessageBuffer, retained);
}

void MqttBase::publish(const char* topic, const char* methode, const char* state, const char* data,
                       bool retained) {
  JSONencoder_["method"] = methode;
  JSONencoder_["state"] = state;
  JSONencoder_["data"] = data;

  char JSONmessageBuffer[100];

  serializeJson(doc_, JSONmessageBuffer, 100);
  debug_print("send message");
  debug_println(JSONencoder_);

  pub_client_->publish(topic, JSONmessageBuffer, retained);
}

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