![PROJECT_PHOTO](https://github.com/AlexGyver/WS2812_painter/blob/master/proj_img.jpg)
# Рисуем картины светом на Arduino и WS2812b
* [Описание проекта](#chapter-0)
* [Папки проекта](#chapter-1)
* [Схемы подключения](#chapter-2)
* [Материалы и компоненты](#chapter-3)
* [Как скачать и прошить](#chapter-4)
* [FAQ](#chapter-5)
* [Полезная информация](#chapter-6)
[![AlexGyver YouTube](http://alexgyver.ru/git_banner.jpg)](https://www.youtube.com/channel/UCgtAOyEQdAyjvm9ATCi_Aig?sub_confirmation=1)

<a id="chapter-0"></a>
## Описание проекта
**Версия 1.1** - добавлена поддержка двух типов энкодеров
Рисовалка картин для фото на длинной выдержке на Arduino и адресной светодиодной ленте WS2812b  
- Читает картинки формата BMP 24 bit с карты памяти
- Выбор картинки энкодером
- Настройка скорости отрисовки
- Настройка яркости
- Подробности в видео: https://youtu.be/nu31By9Phdc

<a id="chapter-1"></a>
## Папки
**ВНИМАНИЕ! Если это твой первый опыт работы с Arduino, читай [инструкцию](#chapter-4)**
- **libraries** - библиотеки проекта. Заменить имеющиеся версии
- **NeoPixel_Painter** - оригинальная прошивка для Arduino, файл в папке открыть в Arduino IDE ([инструкция](#chapter-4))
- **NeoPixel_Painter_gyver** - модицифированная прошивка от меня. Добавлен дисплей и энкодер
- **schemes** - схемы подключения
- **progs** - программы (paint.net, SD format tool)
- **BMPimages** - готовые картинки из видео

<a id="chapter-2"></a>
## Схемы
### Оригинальная
![SCHEME](https://github.com/AlexGyver/WS2812_painter/blob/master/schemes/scheme1.jpg)
### Модификация
![SCHEME](https://github.com/AlexGyver/WS2812_painter/blob/master/schemes/scheme2.jpg)

<a id="chapter-3"></a>
## Материалы и компоненты
### Ссылки оставлены на магазины, с которых я закупаюсь уже не один год
* Arduino NANO 328p – искать
* https://ali.ski/mkYLnM
* https://ali.ski/B04JW
* https://ali.ski/ohvCH
* https://ali.ski/typjt
* Адресная лента (под нарезку)
* https://ali.ski/jPd3v
* https://ali.ski/T4tvWd
* Купить в РФ, 60 свет/метр, 30 свет/метр
* Black PCB / White PCB – цвет подложки ленты, чёрная / белая. В видео была чёрная
* 1m/5m – длина ленты в метрах (чтобы заказать 2 метра, берите два заказа 1m, очевидно)
* 30/60/74/96/100/144 – количество светодиодов на 1 метр ленты. В видео использовалась лента 60 диодов на метр
* IP30 лента без влагозащиты (как на видео)
* IP65 лента покрыта силиконом
* IP67 лента полностью в силиконовом коробе
* Постфикс ECO – лента чуть более низкого качества, меньше меди, на длинной ленте будет сильно проседать яркость
* Модуль для MicroSD https://ali.ski/comubJ
* Энкодер – искать
* https://ali.ski/Jhe2K
* https://ali.ski/OlGeA
* https://ali.ski/g_EAI
* Дисплей – искать
* https://ali.ski/SP7f5
* https://ali.ski/8r3_F
* https://ali.ski/35baVv
* Батарейный отсек под 4 АА
* https://ali.ski/kez97J
* https://ali.ski/Q9ePs
* https://ali.ski/b7bceZ
* Провод монтажный https://ali.ski/JYt-m4
* **Кнопку и резистор можно купить в автозапчастях или радиодеталях!**

## Вам скорее всего пригодится
* [Всё для пайки (паяльники и примочки)](http://alexgyver.ru/all-for-soldering/)
* [Недорогие инструменты](http://alexgyver.ru/my_instruments/)
* [Все существующие модули и сенсоры Arduino](http://alexgyver.ru/arduino_shop/)
* [Электронные компоненты](http://alexgyver.ru/electronics/)
* [Аккумуляторы и зарядные модули](http://alexgyver.ru/18650/)

<a id="chapter-4"></a>
## Как скачать и прошить
* [Первые шаги с Arduino](http://alexgyver.ru/arduino-first/) - ультра подробная статья по началу работы с Ардуино, ознакомиться первым делом!
* Скачать архив с проектом
> На главной странице проекта (где ты читаешь этот текст) вверху справа зелёная кнопка **Clone or download**, вот её жми, там будет **Download ZIP**
* Установить библиотеки в  
`C:\Program Files (x86)\Arduino\libraries\` (Windows x64)  
`C:\Program Files\Arduino\libraries\` (Windows x86)
* Подключить Ардуино к компьютеру
* Запустить файл прошивки (который имеет расширение .ino)
* Настроить IDE (COM порт, модель Arduino, как в статье выше)
* Настроить что нужно по проекту
* Нажать загрузить
* Пользоваться

## Как пользоваться:
* После изменения/добавления изображений на карту нужно запускать с нажатой кнопкой энкодера
	* Будет предложено выбрать яркость, от 5 до 95
* Если запуск прошёл без ошибок, появится надпись Fr и цифра - номер файла
* Поворот энкодера - выбор файла
* Удерживание кнопки энкодера и поворот - настройка скорости анимации
* Номер файла и скорость сохраняются в энергонезависимой памяти

## РАСШИФРОВКА НАДПИСЕЙ НА ДИСПЛЕЕ:
* Strt - (Start) начало работы, инициализация SD карты. Эту надпись можно успеть 
* прочитать только при запуске в режиме загрузки
* PreP - (Perparing) подготовка к преобразованию изображений* 
* LOAd - (Loading) преобразование и загрузка изображений
* read - (Reading) чтение преобразованных изображений с карты
* rdy  - (Ready) готов к работе* 
* Proc - (Processing) показываем анимацию  
НАСТРОЙКИ:
* Br** - (Brightness) настройка яркости (в процентах, 10-95)
* Fr** - (Frame) номер изображения, которое будет показано при нажатии на кнопку
* SP** - (Speed) скорость анимации (в процентах, 0-95)  
ОШИБКИ:
* Sder - (SD error) ошибка подключения SD карты (карта не подключена, карта сломана, карта не подходит)
* FrEr - (Frame Error) ошибка при чтении картинки. Возникает на разных стадиях, подробнее видно с компьютера
* Ferr - (Fatal error) критическая ошибка, работа остановлена. Возникает на разных стадиях, подробнее видно с компьютера

## Настройки в коде
    #define N_LEDS       144    // количество светодиодов (максимум 170 !!!)
    #define CURRENT_MAX 3500    // максимальный ток (авто-ограничение по расчёту яркости)
    
<a id="chapter-5"></a>
## FAQ
### Основные вопросы
В: Как скачать с этого грёбаного сайта?  
О: На главной странице проекта (где ты читаешь этот текст) вверху справа зелёная кнопка **Clone or download**, вот её жми, там будет **Download ZIP**

В: Скачался какой то файл .zip, куда его теперь?  
О: Это архив. Можно открыть стандартными средствами Windows, но думаю у всех на компьютере установлен WinRAR, архив нужно правой кнопкой и извлечь.

В: Я совсем новичок! Что мне делать с Ардуиной, где взять все программы?  
О: Читай и смотри видос http://alexgyver.ru/arduino-first/

В: Компьютер никак не реагирует на подключение Ардуины!  
О: Возможно у тебя зарядный USB кабель, а нужен именно data-кабель, по которому можно данные передавать

В: Ошибка! Скетч не компилируется!  
О: Путь к скетчу не должен содержать кириллицу. Положи его в корень диска.

В: Сколько стоит?  
О: Ничего не продаю.

### Вопросы по этому проекту

<a id="chapter-6"></a>
## Полезная информация
* [Мой сайт](http://alexgyver.ru/)
* [Основной YouTube канал](https://www.youtube.com/channel/UCgtAOyEQdAyjvm9ATCi_Aig?sub_confirmation=1)
* [YouTube канал про Arduino](https://www.youtube.com/channel/UC4axiS76D784-ofoTdo5zOA?sub_confirmation=1)
* [Мои видеоуроки по пайке](https://www.youtube.com/playlist?list=PLOT_HeyBraBuMIwfSYu7kCKXxQGsUKcqR)
* [Мои видеоуроки по Arduino](http://alexgyver.ru/arduino_lessons/)