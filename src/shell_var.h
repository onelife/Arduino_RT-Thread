/***************************************************************************//**
 * @file    shell_var.h
 * @brief   Arduino RT-Thread shell variables
 * @author  onelife <onelife.real[at]gmail.com>
 ******************************************************************************/
#ifdef FINSH_USING_MSH_ONLY
#warning "Shell variables are not available for current CONFIG"
#else
ADD_SHELL_VAR(dummy, dummy variable for finsh, dummy, finsh_type_int)
#endif

/* Please add your variables with the following format:

 ADD_SHELL_VAR(variable_alias, variable description, variable_name, variable_type)
 - variable_alias: alias of variable_name: must be a valid C identifier 
 - variable description: may have space
 - variable_name: must be a valid C identifier 
 - variable_type: must be one of the following types
   - finsh_type_void:       void
   - finsh_type_voidp:      void pointer
   - finsh_type_char:       char
   - finsh_type_uchar:      unsigned char
   - finsh_type_charp:      char pointer
   - finsh_type_short:      short
   - finsh_type_ushort:     unsigned short
   - finsh_type_shortp:     short pointer
   - finsh_type_int:        int
   - finsh_type_uint:       unsigned int
   - finsh_type_intp:       int pointer
   - finsh_type_long:       long
   - finsh_type_ulong:      unsigned long
   - finsh_type_longp:      long pointer

 Example 1: rt_uint32_t led_id; =>
 ADD_SHELL_VAR(id, LED ID, led_id, finsh_type_uint)

 Example 2: rt_uint8_t led_state =>
 ADD_SHELL_VAR(state, LED state, led_state, finsh_type_uchar)
 */
// ADD_SHELL_VAR(id, LED ID, led_id, finsh_type_uint)
// ADD_SHELL_VAR(state, LED state, led_state, finsh_type_uchar)
