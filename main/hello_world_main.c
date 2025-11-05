#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "driver/gpio.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include "esp_sleep.h"
#include "ds18b20.h"   // sua biblioteca existente para DS18B20
#include "lora.h"
#include "nvs_flash.h"
#include "nvs.h"
#include "esp_spiffs.h"
#include "esp_vfs.h"
#include <sys/unistd.h>
#include <sys/stat.h>

/* -------------------- CONFIGURAÇÕES -------------------- */
#define GPIO_TDS_ENERGIA     GPIO_NUM_32     // pino que alimenta sensor TDS
#define DS_POWER_GPIO        GPIO_NUM_25     // pino que alimenta DS18B20 (VDD do sensor)
#define DS_DATA_PIN          14              // DQ do DS18B20 (conforme seu código)
#define CANAL_ADC_TDS        ADC_CHANNEL_0   // ADC1_CHANNEL_0 = GPIO36
#define UNIDADE_ADC_USADA    ADC_UNIT_1

// Tempos / parâmetros
#define TEMPO_ESTABILIZACAO_SENSOR_TDS   10      // segundos para estabilizar o sensor TDS
#define TEMPO_ESTABILIZACAO_DS_MS       800      // ms para conversão completa (12-bit ~750 ms)
#define NUM_AMOSTRAS_TDS                10
#define INTERVALO_ENTRE_AMOSTRAS_MS     20
#define VREF_PADRAO_MV                  1100
#define TEMPO_SONO_S                    60      // deep sleep em segundos
#define TEMPERATURA_PADRAO              18.0f   // fallback caso leitura DS falhe

// Arquivo para salvar pacotes pendentes no SPIFFS
#define PENDENTES_PATH "/spiffs/pendentes.csv"

// Log tag
static const char *TAG = "MARICULTURA";

/* Variáveis ADC */
static adc_oneshot_unit_handle_t adc_handle;
static adc_cali_handle_t adc_cali_handle = NULL;
static bool adc_tem_calibracao = false;

// NVS handle
nvs_handle_t nvs;

static uint32_t packet_id = 0;  // identificador único dos pacotes

/* -------------------- Funções auxiliares LoRa -------------------- */

// Envia pacote com tentativa de confirmação (ACK)
static bool enviar_com_ack(const char *payload, uint32_t packet_id)
{
    const TickType_t timeout_ticks = pdMS_TO_TICKS(3000); // 3 s
    const int max_tentativas = 3;

    for (int tentativa = 1; tentativa <= max_tentativas; tentativa++) {
        ESP_LOGI(TAG, "Tentativa %d de envio...", tentativa);
        lora_send_packet((uint8_t *)payload, strlen(payload));
        ESP_LOGI(TAG, "Pacote enviado -> %s", payload);

        lora_receive(); // modo RX para aguardar ACK
        TickType_t start_time = xTaskGetTickCount();
        bool ack_ok = false;

        while ((xTaskGetTickCount() - start_time) < timeout_ticks) {
            if (lora_received()) {
                uint8_t buffer[64];
                int len = lora_receive_packet(buffer, sizeof(buffer));
                if (len > 0) {
                    buffer[len] = '\0';
                    ESP_LOGI(TAG, "Recebido: %s", buffer);

                    char ack_str[16];
                    snprintf(ack_str, sizeof(ack_str), "ACK:%lu", (unsigned long)packet_id);
                    if (strstr((char *)buffer, ack_str)) {
                        ESP_LOGI(TAG, "ACK recebido para pacote %lu", (unsigned long)packet_id);
                        return true; // sucesso!
                    }
                }
            }
            vTaskDelay(pdMS_TO_TICKS(100));
        }

        ESP_LOGW(TAG, "ACK não recebido (tentativa %d)", tentativa);
    }

    // se chegou aqui, falhou todas as tentativas
    return false;
}

// Salva um pacote pendente (append no arquivo)
static void salvar_pendente_fs(const char *payload) {
    FILE *f = fopen(PENDENTES_PATH, "a");

    if (f == NULL) {
        ESP_LOGE(TAG, "Falha ao abrir pendentes.csv para escrita");
        return;
    }
    fprintf(f, "%s\n", payload);
    fclose(f);
    ESP_LOGW(TAG, "Pacote salvo no arquivo pendentes.csv: %s", payload);
}

// Extrai id em inteiro de um payload
static uint32_t extrair_id_do_payload(const char *payload) {
    uint32_t id = 0;
    const char *p = strstr(payload, "ID:");
    if (p) {
        p += 3;
        id = (uint32_t)strtoul(p, NULL, 10);
    }
    return id;
}

// Lê e envia todos os pendentes armazenados
static void reenviar_pendentes_fs(void)
{
    FILE *f = fopen(PENDENTES_PATH, "r");
    if (f == NULL) {
        ESP_LOGI(TAG, "Nenhum pacote pendente no SPIFFS");
        return;
    }

    const char *tmp_path = "/spiffs/pendentes.tmp";
    FILE *f_tmp = fopen(tmp_path, "w");
    if (f_tmp == NULL) {
        ESP_LOGW(TAG, "Nao foi possivel criar arquivo temporario. Tentando reenviar sem backup.");
    }

    bool qualquer_falha = false;
    char linha[256];

    while (fgets(linha, sizeof(linha), f) != NULL) {
        linha[strcspn(linha, "\r\n")] = 0; // remove \n
        if (strlen(linha) == 0) continue;

        ESP_LOGW(TAG, "Reenviando pacote pendente: %s", linha);

        uint32_t id_do_pacote = extrair_id_do_payload(linha);

        if (!enviar_com_ack(linha, id_do_pacote)) {
            ESP_LOGW(TAG, "Falha ao reenviar. Mantendo no arquivo pendente.");
            qualquer_falha = true;
            if (f_tmp) fprintf(f_tmp, "%s\n", linha); // mantém no temp
        } else {
            ESP_LOGI(TAG, "Reenvio bem-sucedido para ID %lu", (unsigned long)id_do_pacote);
        }
    }
    fclose(f);
    if (f_tmp) fclose(f_tmp);

    if (!qualquer_falha) {
        // todos reenviados -> remove arquivo
        unlink(PENDENTES_PATH);
        if (f_tmp) unlink(tmp_path);
        ESP_LOGI(TAG, "Todos pendentes reenviados — arquivo removido.");
    } else {
        // houve falhas -> substitui arquivo original pelo tmp (contendo os que falharam)
        if (f_tmp) {
            rename(tmp_path, PENDENTES_PATH);
            ESP_LOGI(TAG, "Arquivo pendentes atualizado com os que permaneceram.");
        } else {
            ESP_LOGW(TAG, "Nao foi possivel atualizar pendentes (tmp inexistente).");
        }
    }
}

/* -------------------- Funções auxiliares ADC -------------------- */

// Inicializa calibração ADC (retorna true se calibrado com sucesso)
static bool inicializar_calibracao_adc(adc_unit_t unidade, adc_channel_t canal, adc_atten_t atenuacao)
{
    esp_err_t ret;
    bool calibrado = false;

#if ADC_CALI_SCHEME_CURVE_FITTING_SUPPORTED
    adc_cali_curve_fitting_config_t config_cali = {
        .unit_id = unidade,
        .chan = canal,
        .atten = atenuacao,
        .bitwidth = ADC_BITWIDTH_DEFAULT,
    };
    ret = adc_cali_create_scheme_curve_fitting(&config_cali, &adc_cali_handle);
    if (ret == ESP_OK) calibrado = true;
#endif

#if ADC_CALI_SCHEME_LINE_FITTING_SUPPORTED
    if (!calibrado) {
        adc_cali_line_fitting_config_t config_cali = {
            .unit_id = unidade,
            .atten = atenuacao,
            .bitwidth = ADC_BITWIDTH_DEFAULT,
        };
        ret = adc_cali_create_scheme_line_fitting(&config_cali, &adc_cali_handle);
        if (ret == ESP_OK) calibrado = true;
    }
#endif

    if (calibrado) ESP_LOGI(TAG, "Calibração ADC inicializada");
    else ESP_LOGW(TAG, "Sem calibração ADC - usando Vref padrão");

    return calibrado;
}

/* -------------------- Configuração de pinos e ADC -------------------- */

// Configura GPIOs e inicializa ADC
static void configurar_pinos(void)
{
    ESP_LOGI(TAG, "Configurando pinos...");

    // configura pino que alimenta TDS
    gpio_config_t io_conf_tds = {
        .mode = GPIO_MODE_OUTPUT,
        .pin_bit_mask = 1ULL << GPIO_TDS_ENERGIA,
        .pull_down_en = 0,
        .pull_up_en = 0,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&io_conf_tds);
    gpio_set_level(GPIO_TDS_ENERGIA, 0); // começa desligado

    // configura pino que alimenta DS18B20
    gpio_config_t io_conf_ds = {
        .mode = GPIO_MODE_OUTPUT,
        .pin_bit_mask = 1ULL << DS_POWER_GPIO,
        .pull_down_en = 0,
        .pull_up_en = 0,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&io_conf_ds);
    gpio_set_level(DS_POWER_GPIO, 0); // começa desligado

    // NOTA: a linha de dados DS_DATA_PIN requer resistor pull-up externo (4.7k) ao VDD do DS.
    // Se necessário, habilite pull-up interno (menos recomendado que o resistor fisico).
    gpio_set_direction(DS_DATA_PIN, GPIO_MODE_INPUT); // biblioteca ds deve manipular se necessário

    // Inicializa ADC oneshot
    adc_oneshot_unit_init_cfg_t cfg_adc = {
        .unit_id = UNIDADE_ADC_USADA,
    };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&cfg_adc, &adc_handle));

    adc_oneshot_chan_cfg_t cfg_canal = {
        .atten = ADC_ATTEN_DB_11,
        .bitwidth = ADC_BITWIDTH_DEFAULT,
    };
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc_handle, CANAL_ADC_TDS, &cfg_canal));

    // tenta calibração
    adc_tem_calibracao = inicializar_calibracao_adc(UNIDADE_ADC_USADA, CANAL_ADC_TDS, ADC_ATTEN_DB_11);
}

/* -------------------- Controle DS18B20 -------------------- */

// Liga DS18B20, aguarda estabilização e inicializa biblioteca
static void ligar_ds18b20(void)
{
    ESP_LOGI(TAG, "Ligando DS18B20 (VDD via GPIO)...");
    gpio_set_level(DS_POWER_GPIO, 1);
    // aguarda estabilização + conversão (dependendo da resolução pode demorar até 750ms)
    vTaskDelay(pdMS_TO_TICKS(TEMPO_ESTABILIZACAO_DS_MS));
    // inicializa a lib após energizar (sua lib espera que DQ esteja presente)
    ds18b20_init(DS_DATA_PIN);
}

// Desliga DS18B20
static void desligar_ds18b20(void)
{
    ESP_LOGI(TAG, "Desligando DS18B20");
    gpio_set_level(DS_POWER_GPIO, 0);
}

/* -------------------- Controle TDS -------------------- */

// Liga sensor TDS e aguarda estabilização
static void ligar_sensor_tds(void)
{
    ESP_LOGI(TAG, "Ligando sensor TDS (aguardando %d s)...", TEMPO_ESTABILIZACAO_SENSOR_TDS);
    gpio_set_level(GPIO_TDS_ENERGIA, 1);
    vTaskDelay(pdMS_TO_TICKS(TEMPO_ESTABILIZACAO_SENSOR_TDS * 1000));
}

// Desliga sensor TDS
static void desligar_sensor_tds(void)
{
    ESP_LOGI(TAG, "Desligando sensor TDS");
    gpio_set_level(GPIO_TDS_ENERGIA, 0);
}

/* -------------------- Leitura ADC -------------------- */

// Lê várias amostras do sensor TDS e retorna média
static float ler_sensor_tds(int num_amostras)
{
    int leitura_bruta = 0;
    int soma = 0;

    for (int i = 0; i < num_amostras; i++) {
        ESP_ERROR_CHECK(adc_oneshot_read(adc_handle, CANAL_ADC_TDS, &leitura_bruta));
        soma += leitura_bruta;
        vTaskDelay(pdMS_TO_TICKS(INTERVALO_ENTRE_AMOSTRAS_MS));
    }

    float media = (float)soma / num_amostras;
    ESP_LOGI(TAG, "Leitura média do ADC: %.2f", media);
    return media;
}

/* -------------------- Conversão para ppm (usa temperatura medida) -------------------- */

// Converte leitura ADC do TDS para ppm, usando compensação de temperatura
static float converter_para_ppm(float leitura_adc, float temperatura_comp)
{
    int tensao_mv = 0;

    if (adc_tem_calibracao) {
        adc_cali_raw_to_voltage(adc_cali_handle, (int)leitura_adc, &tensao_mv);
    } else {
        // aproximação quando não há calibração
        float compensacao_adc = 1 + (1 / 3.9f);
        tensao_mv = (int)((leitura_adc / 4095.0f) * VREF_PADRAO_MV * compensacao_adc);
    }

    float tensao_media_v = tensao_mv / 1000.0f; // V
    // coeficiente de compensação baseado na temperatura (substitui constante fixa)
    float coeficiente_compensacao = 1.0f + 0.02f * (temperatura_comp - 25.0f);
    float tensao_compensada = tensao_media_v / coeficiente_compensacao;

    float valor_tds = (133.42f * tensao_compensada * tensao_compensada * tensao_compensada
                    - 255.86f * tensao_compensada * tensao_compensada
                    + 857.39f * tensao_compensada) * 0.5f;

    ESP_LOGI(TAG, "Tensão média = %.3f V", tensao_media_v);
    ESP_LOGI(TAG, "Tensão compensada = %.3f V (temp = %.2f°C)", tensao_compensada, temperatura_comp);
    ESP_LOGI(TAG, "TDS = %.2f ppm", valor_tds);

    return valor_tds;
}

/* -------------------- Função principal -------------------- */

void app_main(void)
{
    // 1) Loga causa do último despertar e inicializa bibliotecas
    ESP_LOGI(TAG, "Causa do último despertar: %d", esp_sleep_get_wakeup_cause());
    nvs_flash_init();


    // 2) configura GPIOs e ADC
    configurar_pinos(); 


    // 3) Ler temperatura primeiro (para usar na compensação do TDS)
    float temperatura_lida = TEMPERATURA_PADRAO;
    ligar_ds18b20();
    // leitura segura: biblioteca pode expor apenas ds18b20_get_temp()
    float temp = ds18b20_get_temp();
    if (temp > -100.0f && temp < 150.0f) { // leitura válida
        temperatura_lida = temp;
        ESP_LOGI(TAG, "Temperatura lida do DS18B20: %.2f °C", temperatura_lida);
    } else {
        ESP_LOGW(TAG, "Leitura DS18B20 inválida (fallback para %.2f °C)", TEMPERATURA_PADRAO);
    }
    desligar_ds18b20();


    // 4) Ler TDS e converter usando temperatura medida
    ligar_sensor_tds();
    float leitura_adc = ler_sensor_tds(NUM_AMOSTRAS_TDS); // média de várias amostras
    float ppm = converter_para_ppm(leitura_adc, temperatura_lida);
    printf("Leitura final -> Temperatura: %.2f °C, TDS: %.2f ppm\n", temperatura_lida, ppm);
    desligar_sensor_tds();


    // 5) Configurar e inicializar LoRa
    ESP_LOGI(TAG, "Inicializando LoRa..."); 
    if (lora_init() == 0) {         // falha na detecção do módulo LoRa
        ESP_LOGE(TAG, "Módulo LoRa não detectado!");
        while (1) vTaskDelay(1);    // trava aqui
    }
    ESP_LOGI(TAG, "LoRa inicializado com sucesso");
    
    lora_set_frequency(915e6);      // 915 MHz
    ESP_LOGI(TAG, "Frequência LoRa definida para 915 MHz");

    lora_enable_crc();              // Habilita CRC para maior confiabilidade
    lora_set_coding_rate(1);        // 
    lora_set_bandwidth(7);          //
    lora_set_spreading_factor(7);   //
    lora_set_sync_word(0x12);       //


    // 6) Inicializa SPIFFS
    ESP_LOGI(TAG, "Montando SPIFFS...");
    esp_vfs_spiffs_conf_t conf = {
        .base_path = "/spiffs",
        .partition_label = NULL,
        .max_files = 5,
        .format_if_mount_failed = true
    };
    esp_err_t ret = esp_vfs_spiffs_register(&conf);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Falha ao montar SPIFFS (%s)", esp_err_to_name(ret));
    } else {
        ESP_LOGI(TAG, "SPIFFS montado com sucesso");
    }


    // 7) Reenvia pacotes pendentes do SPIFFS
    reenviar_pendentes_fs();


    // 8) Monta payload do pacote
    // Gerencia NVS para ID do pacote
    ESP_LOGI(TAG, "Inicializando NVS...");
    nvs_open("storage", NVS_READWRITE, &nvs);
    nvs_get_u32(nvs, "packet_id", &packet_id);   // lê último ID
    packet_id++;                                        // incrementa para novo pacote
    ESP_LOGI(TAG, "Novo packet_id: %lu", (unsigned long)packet_id);
    nvs_set_u32(nvs, "packet_id", packet_id);    // salva novo ID
    nvs_commit(nvs);
    nvs_close(nvs);

    char payload[64];
    snprintf(payload, sizeof(payload),
             "ID:%lu;TEMP:%.2f;TDS:%.2f", 
             (unsigned long)packet_id,
             temperatura_lida, ppm);
    ESP_LOGI(TAG, "Payload do pacote: %s", payload);


    // 9) Envia pacote atual com retransmissão e ACK
    if (!enviar_com_ack(payload, packet_id)) {
        ESP_LOGE(TAG, "Falha ao enviar pacote %lu, salvando no SPIFFS", (unsigned long)packet_id);
        salvar_pendente_fs(payload);
    }


    // 10) Finaliza LoRa
    lora_sleep(); // coloca LoRa em sleep para economizar energia

    // 11) Entrar em deep sleep
    ESP_LOGI(TAG, "Entrando em deep sleep por %d segundos...", TEMPO_SONO_S);
    esp_sleep_enable_timer_wakeup(TEMPO_SONO_S * 1000000ULL);
    esp_deep_sleep_start();
}
