#include "pmsis.h"
#include "cpx.h"
#include "wifi.h"
#include "bsp/bsp.h"
#include "bsp/camera/himax.h"
#include "bsp/buffer.h"

#define IMG_ORIENTATION 0x0101
#define CAM_WIDTH 324
#define CAM_HEIGHT 244
#define IMG_WIDTH 200
#define IMG_HEIGHT 200

void sendToSTM32(void);
void rx_wifi_task(void *parameters);
void send_image_via_wifi(unsigned char *image, uint16_t width, uint16_t height);
int open_pi_camera_himax(struct pi_device *device);
static void capture_done_cb(void *arg);
void camera_task(void *parameters);

static int wifiClientConnected = 0;
static struct pi_device camera;
unsigned char *imgBuff;
static pi_buffer_t buffer;
static SemaphoreHandle_t capture_sem = NULL;

static EventGroupHandle_t evGroup;
#define CAPTURE_DONE_BIT (1 << 0)

pi_buffer_t header;
uint32_t headerSize;
pi_buffer_t footer;
uint32_t footerSize;

uint8_t is_client_connected = 0;

typedef enum
{
    RAW_ENCODING = 0,
    JPEG_ENCODING = 1
} __attribute__((packed)) StreamerMode_t;

typedef struct
{
    uint8_t magic;
    uint16_t width;
    uint16_t height;
    uint8_t depth;
    uint8_t type;
    uint32_t size;
} __attribute__((packed)) img_header_t;

void start(void)
{

    cpxInit();
    cpxEnableFunction(CPX_F_WIFI_CTRL);

    BaseType_t xTask;

    vTaskDelay(2000);
    cpxPrintToConsole(LOG_TO_CRTP, "\n\n*** IAA LAB03 ***\n\n");

    /* Create wifi listener task */
    xTask = xTaskCreate(rx_wifi_task, "rx_wifi_task", configMINIMAL_STACK_SIZE * 2,
                        NULL, tskIDLE_PRIORITY + 1, NULL);

    /* Open and init camera buffer */
    if (open_pi_camera_himax(&camera))
    {
        cpxPrintToConsole(LOG_TO_CRTP, "Failed to open camera\n");
        return;
    }

    pi_buffer_init(&buffer, PI_BUFFER_TYPE_L2, imgBuff);
    pi_buffer_set_format(&buffer, CAM_WIDTH, CAM_HEIGHT, 1, PI_BUFFER_FORMAT_GRAY);

    imgBuff = (unsigned char *)pmsis_l2_malloc(CAM_WIDTH * CAM_HEIGHT);

    /* Create Camera task */
    xTask = xTaskCreate(camera_task, "camera_task", configMINIMAL_STACK_SIZE * 4,
                        NULL, tskIDLE_PRIORITY + 1, NULL);

    while (1)
    {
        sendToSTM32();
        pi_yield();
        vTaskDelay(100);
    }
}

/**
 * @brief transfer UART to stm32
 * - Retrieves FC frequencies
 * - Sends a buffer from gap8 to STM32
 * - Need a program on stm32 to read uart
 */
void sendToSTM32(void)
{

    /* TODO */
    int fcFreq = pi_freq_get(PI_FREQ_DOMAIN_FC);

    // cpxPrintToConsole(LOG_TO_CRTP, "FC Freq = %uHz\n", fcFreq);

    CPXPacket_t packet;

    // Init packet route
    cpxInitRoute(CPX_T_GAP8, CPX_T_STM32, CPX_F_APP, &packet.route);

    // Copy data in packet data
    size_t sizeFreq = sizeof(fcFreq);
    for (size_t i = 0; i < sizeFreq; ++i)
    {
        packet.data[i] = (fcFreq & (0xff << i * 8)) >> i * 8;
    }
    packet.dataLength = sizeFreq;

    // Stop flooding consol with useless msg but you can retrieve frequency by uncommenting this line
    // cpxSendPacketBlocking(&packet);
}

/**
 * @brief Task wifi management
 * be able to:
 * - know if a PC is connected to the drone
 */
void rx_wifi_task(void *parameters)
{
    static CPXPacket_t rxp;
    static WiFiCTRLPacket_t wifip;

    while (1)
    {
        // Wait until recieving wifi controle packet
        cpxReceivePacketBlocking(CPX_F_WIFI_CTRL, &rxp);
        wifip = *((WiFiCTRLPacket_t *)rxp.data);

        // Check it says a client has connected to the drone
        if (wifip.cmd == WIFI_CTRL_STATUS_CLIENT_CONNECTED)
        {
            // Log a msg on client connection
            if (is_client_connected == 0)
                cpxPrintToConsole(LOG_TO_CRTP, "Wifi client status : %u\n", wifip.data[0]);
            
            is_client_connected = wifip.data[0];
            break;
        }
    }

    return;
}

/**
 * @brief create header info struct relative to the image captured by the drone
 */
void createImageHeaderPacket(CPXPacket_t *packet, uint32_t imgSize, StreamerMode_t imgType)
{
    img_header_t *imgHeader = (img_header_t *)packet->data;
    imgHeader->magic = 0xBC;
    imgHeader->width = CAM_WIDTH;
    imgHeader->height = CAM_HEIGHT;
    imgHeader->depth = 1;
    imgHeader->type = imgType;
    imgHeader->size = imgSize;
    packet->dataLength = sizeof(img_header_t);
}

/**
 * @brief transfer WIFI gap8 to PC
 * Send a gap8 buffer to the PC
 * Need python code to receives data on PC
 */
void send_image_via_wifi(unsigned char *image, uint16_t width, uint16_t height)
{
    static CPXPacket_t txp;
    size_t imgSize = width * height * sizeof(unsigned char);

    // Configure route for sending packet
    cpxInitRoute(CPX_T_GAP8, CPX_T_WIFI_HOST, CPX_F_APP, &txp.route);

    // First send information about the image
    createImageHeaderPacket(&txp, imgSize, RAW_ENCODING);
    cpxSendPacketBlocking(&txp);

    // Send image
    sendBufferViaCPXBlocking(&txp, image, imgSize);
}

/**
 * @brief Callback called when a capture is completed by the camera
 * - must release the acquisition task in order to take a new capture
 */
static void capture_done_cb(void *arg)
{
    // Stop camera
    pi_camera_control(&camera, PI_CAMERA_CMD_STOP, 0);
    // Release semaphore which indicates camera task can prepare/send image to host
    xSemaphoreGive(capture_sem);
}

typedef struct {
    size_t height;
    size_t width;
    unsigned char *data;
}img_t;

/**
 * @brief Crop image by taking only inner center pixel
 */
void reshape_img(img_t *input_img, img_t *output_img){
    size_t xOffset = (input_img->width - output_img->width) / 2;
    size_t yOffset = (input_img->height - output_img->height) / 2;

    for (size_t y = 0; y < output_img->height; y++) {
        for (size_t x = 0; x < output_img->width; x++) {
            output_img->data[y * output_img->width + x] = input_img->data[(y + yOffset) * input_img->width + (x + xOffset)];
        }
    }
}

/**
 * @brief Task enabling the acquisition/sending of an image
 * - Set the callback called at the end of a capture
 * - Starts a new capture
 * - Calls the function for sending the image by wifi
 */
void camera_task(void *parameters)
{
    pi_task_t captureDoneTask;
    static pi_buffer_t buffer;
    vTaskDelay(2000);

    // Prepare camera output image buffer 
    cpxPrintToConsole(LOG_TO_CRTP, "Starting camera task...\n");
    uint32_t resolution = CAM_WIDTH * CAM_HEIGHT;
    uint32_t captureSize = resolution * sizeof(unsigned char);
    imgBuff = (unsigned char *)pmsis_l2_malloc(captureSize);
    if (imgBuff == NULL)
    {
        cpxPrintToConsole(LOG_TO_CRTP, "Failed to allocate Memory for Image \n");
        return;
    }

    // Init camera for control
    if (open_pi_camera_himax(&camera))
    {
        cpxPrintToConsole(LOG_TO_CRTP, "Failed to open camera\n");
        return;
    }

    pi_buffer_init(&buffer, PI_BUFFER_TYPE_L2, imgBuff);
    pi_buffer_set_format(&buffer, CAM_WIDTH, CAM_HEIGHT, 1, PI_BUFFER_FORMAT_GRAY);

    // Init semaphore
    capture_sem = xSemaphoreCreateBinary();

    cpxPrintToConsole(LOG_TO_CRTP, "Starting image fetching...\n");

    // Set callback when camera is stopped
    pi_camera_capture_async(&camera, imgBuff, resolution, pi_task_callback(&captureDoneTask, capture_done_cb, NULL));
    // Start taking picture
    pi_camera_control(&camera, PI_CAMERA_CMD_START, 0);
    // Wait on camera is finished taking photo
    xSemaphoreTake(capture_sem, portMAX_DELAY);

    // Prepare img data for resizing
    img_t input_img = {.width = CAM_WIDTH, .height = CAM_HEIGHT, .data = imgBuff};
    //img_t output_img = {.width = IMG_WIDTH, .height = IMG_HEIGHT};
    //output_img.data = (unsigned char *)pmsis_l2_malloc(IMG_WIDTH*IMG_HEIGHT);

    // Resize image
    reshape_img(&input_img, &output_img);

    // Send cropped image to host if client is actualy connected
    if (is_client_connected)
    {
        cpxPrintToConsole(LOG_TO_CRTP, "Send poopooloopoo...\n");
        send_image_via_wifi(input_img.data, CAM_WIDTH, CAM_HEIGHT);
    }

    // Resources go brrrrrrrrrrrrrrrrrrr
    pmsis_l2_malloc_free(imgBuff, CAM_WIDTH * CAM_HEIGHT);
    pmsis_l2_malloc_free(output_img.data, IMG_HEIGHT*IMG_WIDTH);
}

int open_pi_camera_himax(struct pi_device *device)
{
    struct pi_himax_conf cam_conf;

    pi_himax_conf_init(&cam_conf);

    cam_conf.format = PI_CAMERA_QVGA;

    pi_open_from_conf(device, &cam_conf);
    if (pi_camera_open(device))
        return -1;

    // rotate image
    pi_camera_control(device, PI_CAMERA_CMD_START, 0);
    uint8_t set_value = 3;
    uint8_t reg_value;
    pi_camera_reg_set(device, IMG_ORIENTATION, &set_value);
    pi_time_wait_us(1000000);
    pi_camera_reg_get(device, IMG_ORIENTATION, &reg_value);
    if (set_value != reg_value)
    {
        cpxPrintToConsole(LOG_TO_CRTP, "Failed to rotate camera image\n");
        return -1;
    }
    pi_camera_control(device, PI_CAMERA_CMD_STOP, 0);
    pi_camera_control(device, PI_CAMERA_CMD_AEG_INIT, 0);

    return 0;
}

/* Program Entry. */
int main(void)
{
    pi_bsp_init();

    return pmsis_kickoff((void *)start);
}
