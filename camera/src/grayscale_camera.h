extern void grayscale_setup(void);

extern bool ei_grayscale_init(void);
extern void ei_grayscale_deinit(void);
extern bool ei_grayscale_capture(uint32_t img_width, uint32_t img_height, uint8_t *out_buf);
extern void run_grayscale_ei(void);
