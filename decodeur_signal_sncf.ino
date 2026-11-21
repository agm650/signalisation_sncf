/*
  Décodeur de signal SNCF HO - TrainPilot v2.1.1
  ESP32-WROOM-32D / Arduino Uno R3 et Nano (ATmega328P) / ATtiny85
  Réception DCC par 6N137 - NmraDcc

  SIGNAL_TYPE :
    1 : Vert (haut), Rouge (milieu), Jaune (bas)
    2 : Blanc (haut), Vide, Violet (bas)
    3 : Rouge (haut), Vert (milieu), Rouge (bas)
    4 a 13 : Profils SNCF experimentaux (voir README)
    90 : Banc de test 8 sorties (pas un signal SNCF)
    91 : Banc de test 9 sorties (pas un signal SNCF)

  Paquets DCC : Extended Accessory Decoder Control.

  ATTENTION :
  - Version de prototype non encore validée sur matériel.
  - Le numéro SIGNAL_ADDR doit correspondre à l'adresse
    effectivement configurée dans le décodeur NmraDcc.
*/

#include <Arduino.h>
#include <NmraDcc.h>

#if defined(__AVR__)
  #include <avr/pgmspace.h>
#endif

// -------- Configuration --------

#ifndef SIGNAL_TYPE
  #define SIGNAL_TYPE 1
#endif
#ifndef SIGNAL_ADDR
  #define SIGNAL_ADDR 10
#endif

// 1 : toutes les sorties doivent disposer de PWM materiel.
// 0 : autorise les sorties numeriques sans fondu sur Uno/Nano.
#ifndef REQUIRE_FULL_PWM
  #define REQUIRE_FULL_PWM 1
#endif

// Les profils de banc ne doivent jamais etre utilises comme signaux SNCF.
#ifndef ENABLE_TEST_PROFILES
  #define ENABLE_TEST_PROFILES 0
#endif

// Oeilleton blanc optionnel, uniquement pour les nouveaux profils compatibles.
#ifndef ENABLE_OEILLETON
  #define ENABLE_OEILLETON 0
#endif
#if ENABLE_OEILLETON != 0 && ENABLE_OEILLETON != 1
  #error "ENABLE_OEILLETON doit valoir 0 ou 1"
#endif
#if ENABLE_OEILLETON && !((SIGNAL_TYPE >= 4 && SIGNAL_TYPE <= 9) || SIGNAL_TYPE == 13)
  #error "Oeilleton non disponible pour ce profil"
#endif
#if ENABLE_OEILLETON
  #define EYE_COLUMN(mode) , mode
#else
  #define EYE_COLUMN(mode)
#endif

// Diagnostic uniquement sur Uno / Nano / ESP32.
#ifndef DEBUG_DCC
  #define DEBUG_DCC 1
#endif

#if (REQUIRE_FULL_PWM != 0) && (REQUIRE_FULL_PWM != 1)
  #error "REQUIRE_FULL_PWM doit valoir 0 ou 1"
#endif

#if (ENABLE_TEST_PROFILES != 0) && (ENABLE_TEST_PROFILES != 1)
  #error "ENABLE_TEST_PROFILES doit valoir 0 ou 1"
#endif

#if defined(ESP32) || defined(ARDUINO_ARCH_ESP32)
  #define TARGET_ESP32 1
#else
  #define TARGET_ESP32 0
#endif

#if defined(__AVR_ATtiny85__)
  #define TARGET_ATTINY85 1
#else
  #define TARGET_ATTINY85 0
#endif

// Les sorties PWM sont toujours placees en tete de LED_PINS.
// Les trois premieres broches reprennent exactement le montage initial.
#if TARGET_ESP32
  #if !defined(CONFIG_IDF_TARGET_ESP32)
    #error "ESP32 classique uniquement (WROOM-32 / 32D / 32E)"
  #endif
  constexpr uint8_t DCC_PIN = 27;
  constexpr uint8_t LED_PINS[] = {18, 19, 21, 22, 23, 25, 26, 32, 33};
  constexpr uint8_t MAX_PWM_OUTPUTS = 9;

#elif TARGET_ATTINY85
  // ATTinyCore, numerotation PBx et oscillateur interne.
  constexpr uint8_t DCC_PIN = 2; // PB2 / INT0
  constexpr uint8_t LED_PINS[] = {0, 1, 4};
  constexpr uint8_t MAX_PWM_OUTPUTS = 3;

#elif defined(__AVR_ATmega328P__)
  constexpr uint8_t DCC_PIN = 2;
  constexpr uint8_t LED_PINS[] = {3, 5, 6, 9, 10, 11, 4, 7, 8};
  constexpr uint8_t MAX_PWM_OUTPUTS = 6;

#else
  #error "Plateforme non prise en charge"
#endif

constexpr uint8_t MAX_OUTPUTS = sizeof(LED_PINS) / sizeof(LED_PINS[0]);
static_assert(MAX_PWM_OUTPUTS <= MAX_OUTPUTS, "Profil materiel invalide");
// Limite de conception, independante du microcontroleur.
constexpr uint8_t MAX_SIGNAL_LEDS = 9;

// Verification des conflits de broches (C++11, a la compilation).
constexpr bool pinsValid(uint8_t i, uint8_t j);
constexpr bool pinsValid(uint8_t i, uint8_t j) {
  return i >= MAX_OUTPUTS ? true :
         LED_PINS[i] == DCC_PIN ? false :
         j >= MAX_OUTPUTS ? pinsValid(i + 1, i + 2) :
         LED_PINS[i] == LED_PINS[j] ? false : pinsValid(i, j + 1);
}
static_assert(pinsValid(0, 1), "Conflit de broches LED/DCC");

#if TARGET_ATTINY85
  #define SERIAL_DEBUG_ENABLED 0
#else
  #define SERIAL_DEBUG_ENABLED DEBUG_DCC
#endif

// -------- Aspects --------

enum : uint8_t {
  LED_OFF = 0,
  LED_ON = 1,
  LED_BLINK = 2,
  LED_INVALID = 255 // marqueur de ligne non supportee
};

#if SIGNAL_TYPE == 1

const uint8_t ASPECTS[][3] PROGMEM = {
  {LED_OFF, LED_ON,  LED_OFF}, // 0 : Rouge
  {LED_ON,  LED_OFF, LED_OFF}, // 1 : Vert
  {LED_OFF, LED_OFF, LED_ON }, // 2 : Jaune
  {LED_OFF, LED_OFF, LED_OFF}  // 3 : Eteint
};

#elif SIGNAL_TYPE == 2

const uint8_t ASPECTS[][3] PROGMEM = {
  {LED_OFF,   LED_OFF, LED_ON }, // 0 : Violet
  {LED_ON,    LED_OFF, LED_OFF}, // 1 : Blanc
  {LED_BLINK, LED_OFF, LED_OFF}, // 2 : Blanc clignotant
  {LED_OFF,   LED_OFF, LED_OFF}  // 3 : Eteint
};

#elif SIGNAL_TYPE == 3

const uint8_t ASPECTS[][3] PROGMEM = {
  {LED_ON,  LED_OFF, LED_ON },  // 0 : Carre rouge
  {LED_OFF, LED_ON,  LED_OFF},  // 1 : Vert
  {LED_OFF, LED_OFF, LED_OFF}   // 2 : Eteint
};


#elif SIGNAL_TYPE == 4

// Cible B ; ordre logique : V S A C + O optionnel
const uint8_t ASPECTS[][4 + ENABLE_OEILLETON] PROGMEM = {
  {LED_OFF, LED_ON, LED_OFF, LED_ON EYE_COLUMN(LED_OFF)}, // 0 : Arret (carre / violet / semaphore / disque)
  {LED_ON, LED_OFF, LED_OFF, LED_OFF EYE_COLUMN(LED_ON)}, // 1 : Voie libre
  {LED_OFF, LED_OFF, LED_ON, LED_OFF EYE_COLUMN(LED_ON)}, // 2 : Avertissement
  {LED_OFF, LED_OFF, LED_OFF, LED_OFF EYE_COLUMN(LED_OFF)}, // 3 : Eteint
  {LED_OFF, LED_ON, LED_OFF, LED_OFF EYE_COLUMN(LED_ON)}, // 4 : Semaphore
  {LED_OFF, LED_BLINK, LED_OFF, LED_OFF EYE_COLUMN(LED_ON)}, // 5 : Semaphore clignotant
  {LED_BLINK, LED_OFF, LED_OFF, LED_OFF EYE_COLUMN(LED_ON)}, // 6 : Vert clignotant
  {LED_OFF, LED_OFF, LED_BLINK, LED_OFF EYE_COLUMN(LED_ON)}, // 7 : Jaune clignotant
};

#elif SIGNAL_TYPE == 5

// Cible C ; ordre logique : V S A C M + O optionnel
const uint8_t ASPECTS[][5 + ENABLE_OEILLETON] PROGMEM = {
  {LED_OFF, LED_ON, LED_OFF, LED_ON, LED_OFF EYE_COLUMN(LED_OFF)}, // 0 : Arret (carre / violet / semaphore / disque)
  {LED_ON, LED_OFF, LED_OFF, LED_OFF, LED_OFF EYE_COLUMN(LED_ON)}, // 1 : Voie libre
  {LED_OFF, LED_OFF, LED_ON, LED_OFF, LED_OFF EYE_COLUMN(LED_ON)}, // 2 : Avertissement
  {LED_OFF, LED_OFF, LED_OFF, LED_OFF, LED_OFF EYE_COLUMN(LED_OFF)}, // 3 : Eteint
  {LED_OFF, LED_ON, LED_OFF, LED_OFF, LED_OFF EYE_COLUMN(LED_ON)}, // 4 : Semaphore
  {LED_OFF, LED_BLINK, LED_OFF, LED_OFF, LED_OFF EYE_COLUMN(LED_ON)}, // 5 : Semaphore clignotant
  {LED_BLINK, LED_OFF, LED_OFF, LED_OFF, LED_OFF EYE_COLUMN(LED_ON)}, // 6 : Vert clignotant
  {LED_OFF, LED_OFF, LED_BLINK, LED_OFF, LED_OFF EYE_COLUMN(LED_ON)}, // 7 : Jaune clignotant
  {LED_OFF, LED_OFF, LED_OFF, LED_OFF, LED_ON EYE_COLUMN(LED_OFF)}, // 8 : Manoeuvre blanc
  {LED_OFF, LED_OFF, LED_OFF, LED_OFF, LED_BLINK EYE_COLUMN(LED_OFF)}, // 9 : Manoeuvre blanc clignotant
};

#elif SIGNAL_TYPE == 6

// Cible E ; ordre logique : V S A C H1 H2 + O optionnel
const uint8_t ASPECTS[][6 + ENABLE_OEILLETON] PROGMEM = {
  {LED_OFF, LED_ON, LED_OFF, LED_ON, LED_OFF, LED_OFF EYE_COLUMN(LED_OFF)}, // 0 : Arret (carre / violet / semaphore / disque)
  {LED_ON, LED_OFF, LED_OFF, LED_OFF, LED_OFF, LED_OFF EYE_COLUMN(LED_ON)}, // 1 : Voie libre
  {LED_OFF, LED_OFF, LED_ON, LED_OFF, LED_OFF, LED_OFF EYE_COLUMN(LED_ON)}, // 2 : Avertissement
  {LED_OFF, LED_OFF, LED_OFF, LED_OFF, LED_OFF, LED_OFF EYE_COLUMN(LED_OFF)}, // 3 : Eteint
  {LED_OFF, LED_ON, LED_OFF, LED_OFF, LED_OFF, LED_OFF EYE_COLUMN(LED_ON)}, // 4 : Semaphore
  {LED_OFF, LED_BLINK, LED_OFF, LED_OFF, LED_OFF, LED_OFF EYE_COLUMN(LED_ON)}, // 5 : Semaphore clignotant
  {LED_BLINK, LED_OFF, LED_OFF, LED_OFF, LED_OFF, LED_OFF EYE_COLUMN(LED_ON)}, // 6 : Vert clignotant
  {LED_OFF, LED_OFF, LED_BLINK, LED_OFF, LED_OFF, LED_OFF EYE_COLUMN(LED_ON)}, // 7 : Jaune clignotant
  {LED_INVALID, LED_OFF, LED_OFF, LED_OFF, LED_OFF, LED_OFF EYE_COLUMN(LED_OFF)}, // 8 : Non supporte
  {LED_INVALID, LED_OFF, LED_OFF, LED_OFF, LED_OFF, LED_OFF EYE_COLUMN(LED_OFF)}, // 9 : Non supporte
  {LED_OFF, LED_OFF, LED_OFF, LED_OFF, LED_ON, LED_ON EYE_COLUMN(LED_ON)}, // 10 : Ralentissement 30
  {LED_OFF, LED_OFF, LED_OFF, LED_OFF, LED_BLINK, LED_BLINK EYE_COLUMN(LED_ON)}, // 11 : Ralentissement 60
  {LED_INVALID, LED_OFF, LED_OFF, LED_OFF, LED_OFF, LED_OFF EYE_COLUMN(LED_OFF)}, // 12 : Reserve (combinaison non supportee)
  {LED_INVALID, LED_OFF, LED_OFF, LED_OFF, LED_OFF, LED_OFF EYE_COLUMN(LED_OFF)}, // 13 : Reserve (combinaison non supportee)
  {LED_INVALID, LED_OFF, LED_OFF, LED_OFF, LED_OFF, LED_OFF EYE_COLUMN(LED_OFF)}, // 14 : Non supporte
  {LED_INVALID, LED_OFF, LED_OFF, LED_OFF, LED_OFF, LED_OFF EYE_COLUMN(LED_OFF)}, // 15 : Non supporte
  {LED_INVALID, LED_OFF, LED_OFF, LED_OFF, LED_OFF, LED_OFF EYE_COLUMN(LED_OFF)}, // 16 : Non supporte
  {LED_INVALID, LED_OFF, LED_OFF, LED_OFF, LED_OFF, LED_OFF EYE_COLUMN(LED_OFF)}, // 17 : Non supporte
  {LED_INVALID, LED_OFF, LED_OFF, LED_OFF, LED_OFF, LED_OFF EYE_COLUMN(LED_OFF)}, // 18 : Non supporte
  {LED_INVALID, LED_OFF, LED_OFF, LED_OFF, LED_OFF, LED_OFF EYE_COLUMN(LED_OFF)}, // 19 : Non supporte
  {LED_OFF, LED_OFF, LED_BLINK, LED_OFF, LED_BLINK, LED_BLINK EYE_COLUMN(LED_ON)}, // 20 : R60 + A clignotant
};

#elif SIGNAL_TYPE == 7

// Cible F ; ordre logique : V S A C M H1 H2 + O optionnel
const uint8_t ASPECTS[][7 + ENABLE_OEILLETON] PROGMEM = {
  {LED_OFF, LED_ON, LED_OFF, LED_ON, LED_OFF, LED_OFF, LED_OFF EYE_COLUMN(LED_OFF)}, // 0 : Arret (carre / violet / semaphore / disque)
  {LED_ON, LED_OFF, LED_OFF, LED_OFF, LED_OFF, LED_OFF, LED_OFF EYE_COLUMN(LED_ON)}, // 1 : Voie libre
  {LED_OFF, LED_OFF, LED_ON, LED_OFF, LED_OFF, LED_OFF, LED_OFF EYE_COLUMN(LED_ON)}, // 2 : Avertissement
  {LED_OFF, LED_OFF, LED_OFF, LED_OFF, LED_OFF, LED_OFF, LED_OFF EYE_COLUMN(LED_OFF)}, // 3 : Eteint
  {LED_OFF, LED_ON, LED_OFF, LED_OFF, LED_OFF, LED_OFF, LED_OFF EYE_COLUMN(LED_ON)}, // 4 : Semaphore
  {LED_OFF, LED_BLINK, LED_OFF, LED_OFF, LED_OFF, LED_OFF, LED_OFF EYE_COLUMN(LED_ON)}, // 5 : Semaphore clignotant
  {LED_BLINK, LED_OFF, LED_OFF, LED_OFF, LED_OFF, LED_OFF, LED_OFF EYE_COLUMN(LED_ON)}, // 6 : Vert clignotant
  {LED_OFF, LED_OFF, LED_BLINK, LED_OFF, LED_OFF, LED_OFF, LED_OFF EYE_COLUMN(LED_ON)}, // 7 : Jaune clignotant
  {LED_OFF, LED_OFF, LED_OFF, LED_OFF, LED_ON, LED_OFF, LED_OFF EYE_COLUMN(LED_OFF)}, // 8 : Manoeuvre blanc
  {LED_OFF, LED_OFF, LED_OFF, LED_OFF, LED_BLINK, LED_OFF, LED_OFF EYE_COLUMN(LED_OFF)}, // 9 : Manoeuvre blanc clignotant
  {LED_OFF, LED_OFF, LED_OFF, LED_OFF, LED_OFF, LED_ON, LED_ON EYE_COLUMN(LED_ON)}, // 10 : Ralentissement 30
  {LED_OFF, LED_OFF, LED_OFF, LED_OFF, LED_OFF, LED_BLINK, LED_BLINK EYE_COLUMN(LED_ON)}, // 11 : Ralentissement 60
  {LED_INVALID, LED_OFF, LED_OFF, LED_OFF, LED_OFF, LED_OFF, LED_OFF EYE_COLUMN(LED_OFF)}, // 12 : Reserve (combinaison non supportee)
  {LED_INVALID, LED_OFF, LED_OFF, LED_OFF, LED_OFF, LED_OFF, LED_OFF EYE_COLUMN(LED_OFF)}, // 13 : Reserve (combinaison non supportee)
  {LED_INVALID, LED_OFF, LED_OFF, LED_OFF, LED_OFF, LED_OFF, LED_OFF EYE_COLUMN(LED_OFF)}, // 14 : Non supporte
  {LED_INVALID, LED_OFF, LED_OFF, LED_OFF, LED_OFF, LED_OFF, LED_OFF EYE_COLUMN(LED_OFF)}, // 15 : Non supporte
  {LED_INVALID, LED_OFF, LED_OFF, LED_OFF, LED_OFF, LED_OFF, LED_OFF EYE_COLUMN(LED_OFF)}, // 16 : Non supporte
  {LED_INVALID, LED_OFF, LED_OFF, LED_OFF, LED_OFF, LED_OFF, LED_OFF EYE_COLUMN(LED_OFF)}, // 17 : Non supporte
  {LED_INVALID, LED_OFF, LED_OFF, LED_OFF, LED_OFF, LED_OFF, LED_OFF EYE_COLUMN(LED_OFF)}, // 18 : Non supporte
  {LED_INVALID, LED_OFF, LED_OFF, LED_OFF, LED_OFF, LED_OFF, LED_OFF EYE_COLUMN(LED_OFF)}, // 19 : Non supporte
  {LED_OFF, LED_OFF, LED_BLINK, LED_OFF, LED_OFF, LED_BLINK, LED_BLINK EYE_COLUMN(LED_ON)}, // 20 : R60 + A clignotant
};

#elif SIGNAL_TYPE == 8

// Cible G ; ordre logique : V S A C H1 H2 T1 T2 + O optionnel
const uint8_t ASPECTS[][8 + ENABLE_OEILLETON] PROGMEM = {
  {LED_OFF, LED_ON, LED_OFF, LED_ON, LED_OFF, LED_OFF, LED_OFF, LED_OFF EYE_COLUMN(LED_OFF)}, // 0 : Arret (carre / violet / semaphore / disque)
  {LED_ON, LED_OFF, LED_OFF, LED_OFF, LED_OFF, LED_OFF, LED_OFF, LED_OFF EYE_COLUMN(LED_ON)}, // 1 : Voie libre
  {LED_OFF, LED_OFF, LED_ON, LED_OFF, LED_OFF, LED_OFF, LED_OFF, LED_OFF EYE_COLUMN(LED_ON)}, // 2 : Avertissement
  {LED_OFF, LED_OFF, LED_OFF, LED_OFF, LED_OFF, LED_OFF, LED_OFF, LED_OFF EYE_COLUMN(LED_OFF)}, // 3 : Eteint
  {LED_OFF, LED_ON, LED_OFF, LED_OFF, LED_OFF, LED_OFF, LED_OFF, LED_OFF EYE_COLUMN(LED_ON)}, // 4 : Semaphore
  {LED_OFF, LED_BLINK, LED_OFF, LED_OFF, LED_OFF, LED_OFF, LED_OFF, LED_OFF EYE_COLUMN(LED_ON)}, // 5 : Semaphore clignotant
  {LED_BLINK, LED_OFF, LED_OFF, LED_OFF, LED_OFF, LED_OFF, LED_OFF, LED_OFF EYE_COLUMN(LED_ON)}, // 6 : Vert clignotant
  {LED_OFF, LED_OFF, LED_BLINK, LED_OFF, LED_OFF, LED_OFF, LED_OFF, LED_OFF EYE_COLUMN(LED_ON)}, // 7 : Jaune clignotant
  {LED_INVALID, LED_OFF, LED_OFF, LED_OFF, LED_OFF, LED_OFF, LED_OFF, LED_OFF EYE_COLUMN(LED_OFF)}, // 8 : Non supporte
  {LED_INVALID, LED_OFF, LED_OFF, LED_OFF, LED_OFF, LED_OFF, LED_OFF, LED_OFF EYE_COLUMN(LED_OFF)}, // 9 : Non supporte
  {LED_OFF, LED_OFF, LED_OFF, LED_OFF, LED_ON, LED_ON, LED_OFF, LED_OFF EYE_COLUMN(LED_ON)}, // 10 : Ralentissement 30
  {LED_OFF, LED_OFF, LED_OFF, LED_OFF, LED_BLINK, LED_BLINK, LED_OFF, LED_OFF EYE_COLUMN(LED_ON)}, // 11 : Ralentissement 60
  {LED_INVALID, LED_OFF, LED_OFF, LED_OFF, LED_OFF, LED_OFF, LED_OFF, LED_OFF EYE_COLUMN(LED_OFF)}, // 12 : Reserve (combinaison non supportee)
  {LED_INVALID, LED_OFF, LED_OFF, LED_OFF, LED_OFF, LED_OFF, LED_OFF, LED_OFF EYE_COLUMN(LED_OFF)}, // 13 : Reserve (combinaison non supportee)
  {LED_OFF, LED_OFF, LED_OFF, LED_OFF, LED_OFF, LED_OFF, LED_ON, LED_ON EYE_COLUMN(LED_ON)}, // 14 : Rappel 30
  {LED_OFF, LED_OFF, LED_OFF, LED_OFF, LED_OFF, LED_OFF, LED_BLINK, LED_BLINK EYE_COLUMN(LED_ON)}, // 15 : Rappel 60
  {LED_OFF, LED_OFF, LED_ON, LED_OFF, LED_OFF, LED_OFF, LED_ON, LED_ON EYE_COLUMN(LED_ON)}, // 16 : RR30 + A
  {LED_OFF, LED_OFF, LED_ON, LED_OFF, LED_OFF, LED_OFF, LED_BLINK, LED_BLINK EYE_COLUMN(LED_ON)}, // 17 : RR60 + A
  {LED_OFF, LED_OFF, LED_BLINK, LED_OFF, LED_OFF, LED_OFF, LED_ON, LED_ON EYE_COLUMN(LED_ON)}, // 18 : RR30 + A clignotant
  {LED_OFF, LED_OFF, LED_BLINK, LED_OFF, LED_OFF, LED_OFF, LED_BLINK, LED_BLINK EYE_COLUMN(LED_ON)}, // 19 : RR60 + A clignotant
  {LED_OFF, LED_OFF, LED_BLINK, LED_OFF, LED_BLINK, LED_BLINK, LED_OFF, LED_OFF EYE_COLUMN(LED_ON)}, // 20 : R60 + A clignotant
};

#elif SIGNAL_TYPE == 9

// Cible H ; ordre logique : V S A C M H1 H2 T1 T2 + O optionnel
const uint8_t ASPECTS[][9 + ENABLE_OEILLETON] PROGMEM = {
  {LED_OFF, LED_ON, LED_OFF, LED_ON, LED_OFF, LED_OFF, LED_OFF, LED_OFF, LED_OFF EYE_COLUMN(LED_OFF)}, // 0 : Arret (carre / violet / semaphore / disque)
  {LED_ON, LED_OFF, LED_OFF, LED_OFF, LED_OFF, LED_OFF, LED_OFF, LED_OFF, LED_OFF EYE_COLUMN(LED_ON)}, // 1 : Voie libre
  {LED_OFF, LED_OFF, LED_ON, LED_OFF, LED_OFF, LED_OFF, LED_OFF, LED_OFF, LED_OFF EYE_COLUMN(LED_ON)}, // 2 : Avertissement
  {LED_OFF, LED_OFF, LED_OFF, LED_OFF, LED_OFF, LED_OFF, LED_OFF, LED_OFF, LED_OFF EYE_COLUMN(LED_OFF)}, // 3 : Eteint
  {LED_OFF, LED_ON, LED_OFF, LED_OFF, LED_OFF, LED_OFF, LED_OFF, LED_OFF, LED_OFF EYE_COLUMN(LED_ON)}, // 4 : Semaphore
  {LED_OFF, LED_BLINK, LED_OFF, LED_OFF, LED_OFF, LED_OFF, LED_OFF, LED_OFF, LED_OFF EYE_COLUMN(LED_ON)}, // 5 : Semaphore clignotant
  {LED_BLINK, LED_OFF, LED_OFF, LED_OFF, LED_OFF, LED_OFF, LED_OFF, LED_OFF, LED_OFF EYE_COLUMN(LED_ON)}, // 6 : Vert clignotant
  {LED_OFF, LED_OFF, LED_BLINK, LED_OFF, LED_OFF, LED_OFF, LED_OFF, LED_OFF, LED_OFF EYE_COLUMN(LED_ON)}, // 7 : Jaune clignotant
  {LED_OFF, LED_OFF, LED_OFF, LED_OFF, LED_ON, LED_OFF, LED_OFF, LED_OFF, LED_OFF EYE_COLUMN(LED_OFF)}, // 8 : Manoeuvre blanc
  {LED_OFF, LED_OFF, LED_OFF, LED_OFF, LED_BLINK, LED_OFF, LED_OFF, LED_OFF, LED_OFF EYE_COLUMN(LED_OFF)}, // 9 : Manoeuvre blanc clignotant
  {LED_OFF, LED_OFF, LED_OFF, LED_OFF, LED_OFF, LED_ON, LED_ON, LED_OFF, LED_OFF EYE_COLUMN(LED_ON)}, // 10 : Ralentissement 30
  {LED_OFF, LED_OFF, LED_OFF, LED_OFF, LED_OFF, LED_BLINK, LED_BLINK, LED_OFF, LED_OFF EYE_COLUMN(LED_ON)}, // 11 : Ralentissement 60
  {LED_INVALID, LED_OFF, LED_OFF, LED_OFF, LED_OFF, LED_OFF, LED_OFF, LED_OFF, LED_OFF EYE_COLUMN(LED_OFF)}, // 12 : Reserve (combinaison non supportee)
  {LED_INVALID, LED_OFF, LED_OFF, LED_OFF, LED_OFF, LED_OFF, LED_OFF, LED_OFF, LED_OFF EYE_COLUMN(LED_OFF)}, // 13 : Reserve (combinaison non supportee)
  {LED_OFF, LED_OFF, LED_OFF, LED_OFF, LED_OFF, LED_OFF, LED_OFF, LED_ON, LED_ON EYE_COLUMN(LED_ON)}, // 14 : Rappel 30
  {LED_OFF, LED_OFF, LED_OFF, LED_OFF, LED_OFF, LED_OFF, LED_OFF, LED_BLINK, LED_BLINK EYE_COLUMN(LED_ON)}, // 15 : Rappel 60
  {LED_OFF, LED_OFF, LED_ON, LED_OFF, LED_OFF, LED_OFF, LED_OFF, LED_ON, LED_ON EYE_COLUMN(LED_ON)}, // 16 : RR30 + A
  {LED_OFF, LED_OFF, LED_ON, LED_OFF, LED_OFF, LED_OFF, LED_OFF, LED_BLINK, LED_BLINK EYE_COLUMN(LED_ON)}, // 17 : RR60 + A
  {LED_OFF, LED_OFF, LED_BLINK, LED_OFF, LED_OFF, LED_OFF, LED_OFF, LED_ON, LED_ON EYE_COLUMN(LED_ON)}, // 18 : RR30 + A clignotant
  {LED_OFF, LED_OFF, LED_BLINK, LED_OFF, LED_OFF, LED_OFF, LED_OFF, LED_BLINK, LED_BLINK EYE_COLUMN(LED_ON)}, // 19 : RR60 + A clignotant
  {LED_OFF, LED_OFF, LED_BLINK, LED_OFF, LED_OFF, LED_BLINK, LED_BLINK, LED_OFF, LED_OFF EYE_COLUMN(LED_ON)}, // 20 : R60 + A clignotant
};

#elif SIGNAL_TYPE == 10

// Cible I/J manoeuvre ; ordre logique : M P
const uint8_t ASPECTS[][2] PROGMEM = {
  {LED_OFF, LED_ON}, // 0 : Arret (carre / violet / semaphore / disque)
  {LED_INVALID, LED_OFF}, // 1 : Non supporte
  {LED_INVALID, LED_OFF}, // 2 : Non supporte
  {LED_OFF, LED_OFF}, // 3 : Eteint
  {LED_INVALID, LED_OFF}, // 4 : Non supporte
  {LED_INVALID, LED_OFF}, // 5 : Non supporte
  {LED_INVALID, LED_OFF}, // 6 : Non supporte
  {LED_INVALID, LED_OFF}, // 7 : Non supporte
  {LED_ON, LED_OFF}, // 8 : Manoeuvre blanc
  {LED_BLINK, LED_OFF}, // 9 : Manoeuvre blanc clignotant
};

#elif SIGNAL_TYPE == 11

// Cible A 2 feux ; ordre logique : V S
const uint8_t ASPECTS[][2] PROGMEM = {
  {LED_OFF, LED_ON}, // 0 : Arret (carre / violet / semaphore / disque)
  {LED_ON, LED_OFF}, // 1 : Voie libre
  {LED_INVALID, LED_OFF}, // 2 : Non supporte
  {LED_OFF, LED_OFF}, // 3 : Eteint
  {LED_INVALID, LED_OFF}, // 4 : Non supporte
  {LED_OFF, LED_BLINK}, // 5 : Semaphore clignotant
  {LED_BLINK, LED_OFF}, // 6 : Vert clignotant
};

#elif SIGNAL_TYPE == 12

// Cible R disque ; ordre logique : V S A D H1 H2
const uint8_t ASPECTS[][6] PROGMEM = {
  {LED_OFF, LED_ON, LED_OFF, LED_ON, LED_OFF, LED_OFF}, // 0 : Arret (carre / violet / semaphore / disque)
  {LED_ON, LED_OFF, LED_OFF, LED_OFF, LED_OFF, LED_OFF}, // 1 : Voie libre
  {LED_OFF, LED_OFF, LED_ON, LED_OFF, LED_OFF, LED_OFF}, // 2 : Avertissement
  {LED_OFF, LED_OFF, LED_OFF, LED_OFF, LED_OFF, LED_OFF}, // 3 : Eteint
  {LED_INVALID, LED_OFF, LED_OFF, LED_OFF, LED_OFF, LED_OFF}, // 4 : Non supporte
  {LED_INVALID, LED_OFF, LED_OFF, LED_OFF, LED_OFF, LED_OFF}, // 5 : Non supporte
  {LED_INVALID, LED_OFF, LED_OFF, LED_OFF, LED_OFF, LED_OFF}, // 6 : Non supporte
  {LED_OFF, LED_OFF, LED_BLINK, LED_OFF, LED_OFF, LED_OFF}, // 7 : Jaune clignotant
  {LED_INVALID, LED_OFF, LED_OFF, LED_OFF, LED_OFF, LED_OFF}, // 8 : Non supporte
  {LED_INVALID, LED_OFF, LED_OFF, LED_OFF, LED_OFF, LED_OFF}, // 9 : Non supporte
  {LED_OFF, LED_OFF, LED_OFF, LED_OFF, LED_ON, LED_ON}, // 10 : Ralentissement 30
  {LED_OFF, LED_OFF, LED_OFF, LED_OFF, LED_BLINK, LED_BLINK}, // 11 : Ralentissement 60
  {LED_INVALID, LED_OFF, LED_OFF, LED_OFF, LED_OFF, LED_OFF}, // 12 : Reserve (combinaison non supportee)
  {LED_INVALID, LED_OFF, LED_OFF, LED_OFF, LED_OFF, LED_OFF}, // 13 : Reserve (combinaison non supportee)
  {LED_INVALID, LED_OFF, LED_OFF, LED_OFF, LED_OFF, LED_OFF}, // 14 : Non supporte
  {LED_INVALID, LED_OFF, LED_OFF, LED_OFF, LED_OFF, LED_OFF}, // 15 : Non supporte
  {LED_INVALID, LED_OFF, LED_OFF, LED_OFF, LED_OFF, LED_OFF}, // 16 : Non supporte
  {LED_INVALID, LED_OFF, LED_OFF, LED_OFF, LED_OFF, LED_OFF}, // 17 : Non supporte
  {LED_INVALID, LED_OFF, LED_OFF, LED_OFF, LED_OFF, LED_OFF}, // 18 : Non supporte
  {LED_INVALID, LED_OFF, LED_OFF, LED_OFF, LED_OFF, LED_OFF}, // 19 : Non supporte
  {LED_OFF, LED_OFF, LED_BLINK, LED_OFF, LED_BLINK, LED_BLINK}, // 20 : R60 + A clignotant
};

#elif SIGNAL_TYPE == 13

// Cible A 3 feux etendu ; ordre logique : V S A + O optionnel
const uint8_t ASPECTS[][3 + ENABLE_OEILLETON] PROGMEM = {
  {LED_OFF, LED_ON, LED_OFF EYE_COLUMN(LED_ON)}, // 0 : Arret (carre / violet / semaphore / disque)
  {LED_ON, LED_OFF, LED_OFF EYE_COLUMN(LED_ON)}, // 1 : Voie libre
  {LED_OFF, LED_OFF, LED_ON EYE_COLUMN(LED_ON)}, // 2 : Avertissement
  {LED_OFF, LED_OFF, LED_OFF EYE_COLUMN(LED_OFF)}, // 3 : Eteint
  {LED_INVALID, LED_OFF, LED_OFF EYE_COLUMN(LED_OFF)}, // 4 : Non supporte
  {LED_OFF, LED_BLINK, LED_OFF EYE_COLUMN(LED_ON)}, // 5 : Semaphore clignotant
  {LED_BLINK, LED_OFF, LED_OFF EYE_COLUMN(LED_ON)}, // 6 : Vert clignotant
  {LED_OFF, LED_OFF, LED_BLINK EYE_COLUMN(LED_ON)}, // 7 : Jaune clignotant
};

#elif SIGNAL_TYPE == 90

#if !ENABLE_TEST_PROFILES
  #error "Profil de banc : definir ENABLE_TEST_PROFILES a 1"
#endif
// Banc de test uniquement : aspect 0 = toutes les sorties eteintes.
const uint8_t ASPECTS[][8] PROGMEM = {
  {LED_OFF, LED_OFF, LED_OFF, LED_OFF, LED_OFF, LED_OFF, LED_OFF, LED_OFF}, // 0 : Tout eteint
  {LED_ON, LED_OFF, LED_OFF, LED_OFF, LED_OFF, LED_OFF, LED_OFF, LED_OFF}, // 1 : Sortie 1 seule
  {LED_OFF, LED_ON, LED_OFF, LED_OFF, LED_OFF, LED_OFF, LED_OFF, LED_OFF}, // 2 : Sortie 2 seule
  {LED_OFF, LED_OFF, LED_ON, LED_OFF, LED_OFF, LED_OFF, LED_OFF, LED_OFF}, // 3 : Sortie 3 seule
  {LED_OFF, LED_OFF, LED_OFF, LED_ON, LED_OFF, LED_OFF, LED_OFF, LED_OFF}, // 4 : Sortie 4 seule
  {LED_OFF, LED_OFF, LED_OFF, LED_OFF, LED_ON, LED_OFF, LED_OFF, LED_OFF}, // 5 : Sortie 5 seule
  {LED_OFF, LED_OFF, LED_OFF, LED_OFF, LED_OFF, LED_ON, LED_OFF, LED_OFF}, // 6 : Sortie 6 seule
  {LED_OFF, LED_OFF, LED_OFF, LED_OFF, LED_OFF, LED_OFF, LED_ON, LED_OFF}, // 7 : Sortie 7 seule
  {LED_OFF, LED_OFF, LED_OFF, LED_OFF, LED_OFF, LED_OFF, LED_OFF, LED_ON}, // 8 : Sortie 8 seule
  {LED_ON, LED_ON, LED_ON, LED_ON, LED_ON, LED_ON, LED_ON, LED_ON}, // 9 : Tout allume
  {LED_BLINK, LED_BLINK, LED_BLINK, LED_BLINK, LED_BLINK, LED_BLINK, LED_BLINK, LED_BLINK}, // 10 : Tout clignotant
  {LED_ON, LED_BLINK, LED_OFF, LED_ON, LED_BLINK, LED_OFF, LED_ON, LED_BLINK}  // 11 : Mixte
};

#elif SIGNAL_TYPE == 91

#if !ENABLE_TEST_PROFILES
  #error "Profil de banc : definir ENABLE_TEST_PROFILES a 1"
#endif
// Banc de test uniquement : aspect 0 = toutes les sorties eteintes.
const uint8_t ASPECTS[][9] PROGMEM = {
  {LED_OFF, LED_OFF, LED_OFF, LED_OFF, LED_OFF, LED_OFF, LED_OFF, LED_OFF, LED_OFF}, // 0 : Tout eteint
  {LED_ON, LED_OFF, LED_OFF, LED_OFF, LED_OFF, LED_OFF, LED_OFF, LED_OFF, LED_OFF}, // 1 : Sortie 1 seule
  {LED_OFF, LED_ON, LED_OFF, LED_OFF, LED_OFF, LED_OFF, LED_OFF, LED_OFF, LED_OFF}, // 2 : Sortie 2 seule
  {LED_OFF, LED_OFF, LED_ON, LED_OFF, LED_OFF, LED_OFF, LED_OFF, LED_OFF, LED_OFF}, // 3 : Sortie 3 seule
  {LED_OFF, LED_OFF, LED_OFF, LED_ON, LED_OFF, LED_OFF, LED_OFF, LED_OFF, LED_OFF}, // 4 : Sortie 4 seule
  {LED_OFF, LED_OFF, LED_OFF, LED_OFF, LED_ON, LED_OFF, LED_OFF, LED_OFF, LED_OFF}, // 5 : Sortie 5 seule
  {LED_OFF, LED_OFF, LED_OFF, LED_OFF, LED_OFF, LED_ON, LED_OFF, LED_OFF, LED_OFF}, // 6 : Sortie 6 seule
  {LED_OFF, LED_OFF, LED_OFF, LED_OFF, LED_OFF, LED_OFF, LED_ON, LED_OFF, LED_OFF}, // 7 : Sortie 7 seule
  {LED_OFF, LED_OFF, LED_OFF, LED_OFF, LED_OFF, LED_OFF, LED_OFF, LED_ON, LED_OFF}, // 8 : Sortie 8 seule
  {LED_OFF, LED_OFF, LED_OFF, LED_OFF, LED_OFF, LED_OFF, LED_OFF, LED_OFF, LED_ON}, // 9 : Sortie 9 seule
  {LED_ON, LED_ON, LED_ON, LED_ON, LED_ON, LED_ON, LED_ON, LED_ON, LED_ON}, // 10 : Tout allume
  {LED_BLINK, LED_BLINK, LED_BLINK, LED_BLINK, LED_BLINK, LED_BLINK, LED_BLINK, LED_BLINK, LED_BLINK}, // 11 : Tout clignotant
  {LED_ON, LED_BLINK, LED_OFF, LED_ON, LED_BLINK, LED_OFF, LED_ON, LED_BLINK, LED_OFF}  // 12 : Mixte
};

#else
  #error "SIGNAL_TYPE invalide"
#endif

constexpr size_t ASPECT_COUNT = sizeof(ASPECTS) / sizeof(ASPECTS[0]);
constexpr size_t LED_COUNT = sizeof(ASPECTS[0]) / sizeof(ASPECTS[0][0]);
static_assert(ASPECT_COUNT > 0 && ASPECT_COUNT <= 256, "Nombre d aspects invalide (1 a 256)");
static_assert(LED_COUNT > 0 && LED_COUNT <= MAX_SIGNAL_LEDS,
              "Nombre de sorties invalide (1 a 9)");
static_assert(LED_COUNT <= MAX_OUTPUTS,
              "Nombre de sorties incompatible avec la plateforme");
#if REQUIRE_FULL_PWM
static_assert(LED_COUNT <= MAX_PWM_OUTPUTS,
              "PWM insuffisant : choisir ESP32 ou REQUIRE_FULL_PWM=0");
#endif
constexpr size_t NB_ASPECTS = ASPECT_COUNT;
constexpr uint8_t NB_LEDS = LED_COUNT;

// -------- Reglages --------

constexpr uint32_t BLINK_MS = 600;
constexpr uint32_t FADE_TICK_MS = 5;
constexpr uint8_t FADE_STEP = 8;

// -------- Etat --------

NmraDcc Dcc;

uint8_t currentAspect = 0;
uint8_t levels[NB_LEDS] = {};

uint32_t lastFade = 0;
uint32_t lastBlink = 0;
bool blinkOn = true;

// -------- Lecture de la table --------

uint8_t readAspect(uint8_t aspect, uint8_t led) {
#if defined(__AVR__)
  return pgm_read_byte(&ASPECTS[aspect][led]);
#else
  return ASPECTS[aspect][led];
#endif
}

uint8_t targetFor(uint8_t led) {
  const uint8_t mode = readAspect(currentAspect, led);

  if (mode == LED_ON) {
    return 255;
  }

  if (mode == LED_BLINK && blinkOn) {
    return 255;
  }

  return 0;
}

// -------- Sorties lumineuses --------

bool ledHasPwm(uint8_t led) {
  return led < MAX_PWM_OUTPUTS;
}

void writeLed(uint8_t led, uint8_t level) {
  if (ledHasPwm(led)) {
    analogWrite(LED_PINS[led], level);
  } else {
    digitalWrite(LED_PINS[led], level == 0 ? LOW : HIGH);
  }
}

void setupPwm() {
  for (uint8_t i = 0; i < NB_LEDS; i++) {
    const uint8_t pin = LED_PINS[i];

    pinMode(pin, OUTPUT);

#if TARGET_ESP32
    // Arduino-ESP32 3.x :
    // configuration PWM propre à chaque GPIO
    analogWriteResolution(pin, 8);
    analogWriteFrequency(pin, 1000);
#endif

    writeLed(i, 0);
  }
}

// -------- Affichage immediat --------

void applyAspectImmediately() {
  for (uint8_t i = 0; i < NB_LEDS; i++) {
    levels[i] = targetFor(i);
    writeLed(i, levels[i]);
  }
}

// -------- Reception DCC --------

void notifyDccSigOutputState(
    uint16_t addr,
    uint8_t state
) {
#if SERIAL_DEBUG_ENABLED
  Serial.print("DCC signal addr=");
  Serial.print(addr);
  Serial.print(" aspect=");
  Serial.println(state);
#endif

  if (addr != SIGNAL_ADDR) {
    return;
  }

  if (state >= NB_ASPECTS || readAspect(state, 0) == LED_INVALID) {
#if SERIAL_DEBUG_ENABLED
    Serial.println("Aspect inconnu - ignore");
#endif
    return;
  }

  currentAspect = state;
}

// Callback facultatif pour diagnostic des paquets bruts
#if SERIAL_DEBUG_ENABLED
void notifyDccMsg(DCC_MSG *msg) {
  Serial.print("DCC RAW:");

  for (uint8_t i = 0; i < msg->Size; i++) {
    Serial.print(' ');
    if (msg->Data[i] < 16) {
      Serial.print('0');
    }
    Serial.print(msg->Data[i], HEX);
  }

  Serial.println();
}
#endif

// -------- Animation LED --------

void updateLeds() {
  const uint32_t now = millis();

  if (now - lastBlink >= BLINK_MS) {
    lastBlink = now;
    blinkOn = !blinkOn;
  }

  if (now - lastFade < FADE_TICK_MS) {
    return;
  }

  lastFade = now;

  for (uint8_t i = 0; i < NB_LEDS; i++) {
    const int16_t target = targetFor(i);
    // Une sortie numerique commute directement, sans niveau intermediaire.
    if (!ledHasPwm(i)) {
      if (levels[i] != target) {
        levels[i] = static_cast<uint8_t>(target);
        writeLed(i, levels[i]);
      }
      continue;
    }
    int16_t value = levels[i];

    if (value < target) {
      value += FADE_STEP;
      if (value > target) value = target;
    } else if (value > target) {
      value -= FADE_STEP;
      if (value < target) value = target;
    }

    if (levels[i] != value) {
      levels[i] = static_cast<uint8_t>(value);
      writeLed(i, levels[i]);
    }
  }
}

// -------- Initialisation --------

void setup() {
#if SERIAL_DEBUG_ENABLED
  Serial.begin(115200);
  delay(300);

  Serial.println();
  Serial.println("TrainPilot - DCC Signal Decoder");

#if TARGET_ESP32
  Serial.println("Plateforme : ESP32");
#elif TARGET_ATTINY85
  Serial.println("Plateforme : ATtiny85");
#else
  Serial.println("Plateforme : ATmega328P (Uno / Nano)");
#endif

  Serial.print("Signal type : ");
  Serial.println(SIGNAL_TYPE);
  Serial.print("Adresse cible : ");
  Serial.println(SIGNAL_ADDR);
  Serial.print("Sorties utilisees : ");
  Serial.println(NB_LEDS);
  Serial.print("Sorties PWM utilisees : ");
  Serial.println(NB_LEDS < MAX_PWM_OUTPUTS ? NB_LEDS : MAX_PWM_OUTPUTS);
#if SIGNAL_TYPE >= 4 && SIGNAL_TYPE <= 13
  Serial.println("ATTENTION : profil SNCF theorique, a valider sur banc");
#endif
#if SIGNAL_TYPE == 90 || SIGNAL_TYPE == 91
  Serial.println("ATTENTION : profil de banc, pas un signal SNCF");
#endif
#endif

  setupPwm();

  // Aspect initial de securite
  currentAspect = 0;
  applyAspectImmediately();

  // Le 6N137 dispose d'un pull-up EXTERNE :
  // ESP32 : sortie 6N137 -> pull-up vers 3,3 V
  // Nano / ATtiny85 : pull-up vers 5 V

  Dcc.pin(
      digitalPinToInterrupt(DCC_PIN),
      DCC_PIN,
      0
  );

  Dcc.init(
      MAN_ID_DIY,
      10,
      FLAGS_DCC_ACCESSORY_DECODER |
      FLAGS_OUTPUT_ADDRESS_MODE,
      0
  );

#if SERIAL_DEBUG_ENABLED
  Serial.println("NmraDcc initialise");
  Serial.println("En attente de paquets DCC...");
#endif

  lastFade = lastBlink = millis();
}

// -------- Boucle principale --------

void loop() {
  Dcc.process();
  updateLeds();
}
