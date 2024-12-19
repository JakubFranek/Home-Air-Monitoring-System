#ifdef __cplusplus
extern "C"
{
#endif

    void setup_i2c_bus(void);
    void setup_sht4x(void);
    void measure_sht4x(void);
    void setup_sgp41(void);
    void measure_sgp41(void);

#ifdef __cplusplus
}
#endif