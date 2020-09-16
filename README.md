# STM32 HAL MPU9250

- STM32F103C8 72MHz board
- SPI
- Check `Mpu9250::init`, `Mpu9250::set_read_only_mode`

# Usage

```shell
Core
├── Inc
│   ├── mpu9250_reg_map.h
│   ├── mpu9250.h
│   └── ...
├── Src
│   ├── mpu9250.cpp
│   └── ...
└── ...
```

```cpp
...

/* USER CODE BEGIN Includes */
#include "mpu9250.h"

#include <stdio.h>
/* USER CODE END Includes */

...

int main(void) {

    ...

    /* USER CODE BEGIN 2 */
    lot::Mpu9250 mpu(&hspi1, GPIOA, GPIO_PIN_4);
    if(mpu.init() != HAL_OK) { printf("Failed to init MPU9250.\n"); }

    printf("Calibrating ACC, GYRO ...\n");
    mpu.calibrate_acc();
    mpu.calibrate_gyro();

    printf("Calibrating MAG ...\n");
    mpu.calibrate_mag();

    printf("Finish.\n");
    /* USER CODE END 2 */

    ...

    /* USER CODE BEGIN WHILE */
    while(1) {
        int16_t xyz[3];
        mpu.get_mag(xyz);
        printf("%d,%d,%d\n", xyz[0], xyz[1], xyz[2]);
        HAL_Delay(50);
        /* USER CODE END WHILE */

        ...

    }

    ...

}

...
```
