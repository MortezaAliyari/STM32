# STM32
##STM32F746ZG
###Nucleo Boards
In this project, two ADC channel read two different analogue channels simultaneously by stm32F746 with 1MSPS.
Its ADC sample rate was 1MSPS that was sent to the server system by its DMA Ethernet (LWIP lib). Online monitoring on the server shows that ADC unit self-calibrating has a significant effect on the accuracy of the conversion. So there is no need for an external ADC converter IC and this will lead to reducing product cost.
![Capture](https://user-images.githubusercontent.com/79801785/121144401-33183f80-c853-11eb-9e1f-7b3e7059a532.JPG)


