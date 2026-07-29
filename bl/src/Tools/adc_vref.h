#ifndef adc_cali_h
#define adc_cali_h

#ifdef __cplusplus
extern "C" {
#endif
#if (INGCHIPS_FAMILY == INGCHIPS_FAMILY_20)
/**
 ****************************************************************************************
 * @brief Prepare ADC calibration
 *
 *  Note: if any calibration method has been done, then the method is selected;
 *        otherwise, do self-calibration, then reset.
 *
 * @return              Selected calibration method
 ****************************************************************************************
 */
void adc_verf_enable(void);

/**
 ****************************************************************************************
 * @brief Calibrate raw ADC readings
 *
 *  Note: `adc_prepare_calibration` must be called before using this.
 *
 * @param[in]  mode             sample mode
 * @param[in]  channel_id       ADC channel ID of the value
 * @param[in]  value            raw ADC reading
 * @return                      calibrated ADC value
 ****************************************************************************************
 */
void adc_verf_disable(void);
#endif

#ifdef __cplusplus
}
#endif

#endif
