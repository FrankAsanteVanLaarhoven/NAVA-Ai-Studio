// Localization System for NAVΛ Studio
// Supports multiple languages with automatic detection

export interface LocalizationStrings {
  // Navigation
  workspace: string;
  rosLearning: string;
  fullIDE: string;
  downloads: string;
  documentation: string;
  aiAssistant: string;
  
  // Time & Date
  time: string;
  date: string;
  timezone: string;
  
  // Weather
  weather: string;
  temperature: string;
  humidity: string;
  windSpeed: string;
  forecast: string;
  today: string;
  tomorrow: string;
  
  // Currency
  currency: string;
  currencyConverter: string;
  from: string;
  to: string;
  amount: string;
  convert: string;
  exchangeRate: string;
  
  // Settings
  settings: string;
  language: string;
  location: string;
  preferences: string;
  
  // Common
  loading: string;
  error: string;
  success: string;
  close: string;
  save: string;
  cancel: string;
  
  // Days of week
  monday: string;
  tuesday: string;
  wednesday: string;
  thursday: string;
  friday: string;
  saturday: string;
  sunday: string;
  
  // Months
  january: string;
  february: string;
  march: string;
  april: string;
  may: string;
  june: string;
  july: string;
  august: string;
  september: string;
  october: string;
  november: string;
  december: string;
}

export const translations: Record<string, LocalizationStrings> = {
  en: {
    workspace: 'Workspace',
    rosLearning: 'ROS Learning',
    fullIDE: 'Full IDE',
    downloads: 'Downloads',
    documentation: 'Documentation',
    aiAssistant: 'AI Assistant',
    time: 'Time',
    date: 'Date',
    timezone: 'Timezone',
    weather: 'Weather',
    temperature: 'Temperature',
    humidity: 'Humidity',
    windSpeed: 'Wind Speed',
    forecast: 'Forecast',
    today: 'Today',
    tomorrow: 'Tomorrow',
    currency: 'Currency',
    currencyConverter: 'Currency Converter',
    from: 'From',
    to: 'To',
    amount: 'Amount',
    convert: 'Convert',
    exchangeRate: 'Exchange Rate',
    settings: 'Settings',
    language: 'Language',
    location: 'Location',
    preferences: 'Preferences',
    loading: 'Loading...',
    error: 'Error',
    success: 'Success',
    close: 'Close',
    save: 'Save',
    cancel: 'Cancel',
    monday: 'Monday',
    tuesday: 'Tuesday',
    wednesday: 'Wednesday',
    thursday: 'Thursday',
    friday: 'Friday',
    saturday: 'Saturday',
    sunday: 'Sunday',
    january: 'January',
    february: 'February',
    march: 'March',
    april: 'April',
    may: 'May',
    june: 'June',
    july: 'July',
    august: 'August',
    september: 'September',
    october: 'October',
    november: 'November',
    december: 'December',
  },
  es: {
    workspace: 'Espacio de trabajo',
    rosLearning: 'Aprendizaje ROS',
    fullIDE: 'IDE Completo',
    downloads: 'Descargas',
    documentation: 'Documentación',
    aiAssistant: 'Asistente IA',
    time: 'Hora',
    date: 'Fecha',
    timezone: 'Zona horaria',
    weather: 'Clima',
    temperature: 'Temperatura',
    humidity: 'Humedad',
    windSpeed: 'Velocidad del viento',
    forecast: 'Pronóstico',
    today: 'Hoy',
    tomorrow: 'Mañana',
    currency: 'Moneda',
    currencyConverter: 'Conversor de moneda',
    from: 'De',
    to: 'A',
    amount: 'Cantidad',
    convert: 'Convertir',
    exchangeRate: 'Tipo de cambio',
    settings: 'Configuración',
    language: 'Idioma',
    location: 'Ubicación',
    preferences: 'Preferencias',
    loading: 'Cargando...',
    error: 'Error',
    success: 'Éxito',
    close: 'Cerrar',
    save: 'Guardar',
    cancel: 'Cancelar',
    monday: 'Lunes',
    tuesday: 'Martes',
    wednesday: 'Miércoles',
    thursday: 'Jueves',
    friday: 'Viernes',
    saturday: 'Sábado',
    sunday: 'Domingo',
    january: 'Enero',
    february: 'Febrero',
    march: 'Marzo',
    april: 'Abril',
    may: 'Mayo',
    june: 'Junio',
    july: 'Julio',
    august: 'Agosto',
    september: 'Septiembre',
    october: 'Octubre',
    november: 'Noviembre',
    december: 'Diciembre',
  },
  fr: {
    workspace: 'Espace de travail',
    rosLearning: 'Apprentissage ROS',
    fullIDE: 'IDE Complet',
    downloads: 'Téléchargements',
    documentation: 'Documentation',
    aiAssistant: 'Assistant IA',
    time: 'Heure',
    date: 'Date',
    timezone: 'Fuseau horaire',
    weather: 'Météo',
    temperature: 'Température',
    humidity: 'Humidité',
    windSpeed: 'Vitesse du vent',
    forecast: 'Prévisions',
    today: "Aujourd'hui",
    tomorrow: 'Demain',
    currency: 'Devise',
    currencyConverter: 'Convertisseur de devises',
    from: 'De',
    to: 'À',
    amount: 'Montant',
    convert: 'Convertir',
    exchangeRate: 'Taux de change',
    settings: 'Paramètres',
    language: 'Langue',
    location: 'Emplacement',
    preferences: 'Préférences',
    loading: 'Chargement...',
    error: 'Erreur',
    success: 'Succès',
    close: 'Fermer',
    save: 'Enregistrer',
    cancel: 'Annuler',
    monday: 'Lundi',
    tuesday: 'Mardi',
    wednesday: 'Mercredi',
    thursday: 'Jeudi',
    friday: 'Vendredi',
    saturday: 'Samedi',
    sunday: 'Dimanche',
    january: 'Janvier',
    february: 'Février',
    march: 'Mars',
    april: 'Avril',
    may: 'Mai',
    june: 'Juin',
    july: 'Juillet',
    august: 'Août',
    september: 'Septembre',
    october: 'Octobre',
    november: 'Novembre',
    december: 'Décembre',
  },
  de: {
    workspace: 'Arbeitsbereich',
    rosLearning: 'ROS Lernen',
    fullIDE: 'Vollständige IDE',
    downloads: 'Downloads',
    documentation: 'Dokumentation',
    aiAssistant: 'KI-Assistent',
    time: 'Zeit',
    date: 'Datum',
    timezone: 'Zeitzone',
    weather: 'Wetter',
    temperature: 'Temperatur',
    humidity: 'Luftfeuchtigkeit',
    windSpeed: 'Windgeschwindigkeit',
    forecast: 'Vorhersage',
    today: 'Heute',
    tomorrow: 'Morgen',
    currency: 'Währung',
    currencyConverter: 'Währungsrechner',
    from: 'Von',
    to: 'Zu',
    amount: 'Betrag',
    convert: 'Konvertieren',
    exchangeRate: 'Wechselkurs',
    settings: 'Einstellungen',
    language: 'Sprache',
    location: 'Standort',
    preferences: 'Einstellungen',
    loading: 'Laden...',
    error: 'Fehler',
    success: 'Erfolg',
    close: 'Schließen',
    save: 'Speichern',
    cancel: 'Abbrechen',
    monday: 'Montag',
    tuesday: 'Dienstag',
    wednesday: 'Mittwoch',
    thursday: 'Donnerstag',
    friday: 'Freitag',
    saturday: 'Samstag',
    sunday: 'Sonntag',
    january: 'Januar',
    february: 'Februar',
    march: 'März',
    april: 'April',
    may: 'Mai',
    june: 'Juni',
    july: 'Juli',
    august: 'August',
    september: 'September',
    october: 'Oktober',
    november: 'November',
    december: 'Dezember',
  },
  zh: {
    workspace: '工作区',
    rosLearning: 'ROS学习',
    fullIDE: '完整IDE',
    downloads: '下载',
    documentation: '文档',
    aiAssistant: 'AI助手',
    time: '时间',
    date: '日期',
    timezone: '时区',
    weather: '天气',
    temperature: '温度',
    humidity: '湿度',
    windSpeed: '风速',
    forecast: '预报',
    today: '今天',
    tomorrow: '明天',
    currency: '货币',
    currencyConverter: '货币转换器',
    from: '从',
    to: '到',
    amount: '金额',
    convert: '转换',
    exchangeRate: '汇率',
    settings: '设置',
    language: '语言',
    location: '位置',
    preferences: '偏好',
    loading: '加载中...',
    error: '错误',
    success: '成功',
    close: '关闭',
    save: '保存',
    cancel: '取消',
    monday: '星期一',
    tuesday: '星期二',
    wednesday: '星期三',
    thursday: '星期四',
    friday: '星期五',
    saturday: '星期六',
    sunday: '星期日',
    january: '一月',
    february: '二月',
    march: '三月',
    april: '四月',
    may: '五月',
    june: '六月',
    july: '七月',
    august: '八月',
    september: '九月',
    october: '十月',
    november: '十一月',
    december: '十二月',
  },
  ja: {
    workspace: 'ワークスペース',
    rosLearning: 'ROS学習',
    fullIDE: 'フルIDE',
    downloads: 'ダウンロード',
    documentation: 'ドキュメント',
    aiAssistant: 'AIアシスタント',
    time: '時刻',
    date: '日付',
    timezone: 'タイムゾーン',
    weather: '天気',
    temperature: '気温',
    humidity: '湿度',
    windSpeed: '風速',
    forecast: '予報',
    today: '今日',
    tomorrow: '明日',
    currency: '通貨',
    currencyConverter: '通貨換算',
    from: 'から',
    to: 'へ',
    amount: '金額',
    convert: '変換',
    exchangeRate: '為替レート',
    settings: '設定',
    language: '言語',
    location: '場所',
    preferences: '設定',
    loading: '読み込み中...',
    error: 'エラー',
    success: '成功',
    close: '閉じる',
    save: '保存',
    cancel: 'キャンセル',
    monday: '月曜日',
    tuesday: '火曜日',
    wednesday: '水曜日',
    thursday: '木曜日',
    friday: '金曜日',
    saturday: '土曜日',
    sunday: '日曜日',
    january: '1月',
    february: '2月',
    march: '3月',
    april: '4月',
    may: '5月',
    june: '6月',
    july: '7月',
    august: '8月',
    september: '9月',
    october: '10月',
    november: '11月',
    december: '12月',
  },
  ar: {
    workspace: 'مساحة العمل',
    rosLearning: 'تعلم ROS',
    fullIDE: 'IDE كامل',
    downloads: 'التنزيلات',
    documentation: 'التوثيق',
    aiAssistant: 'مساعد الذكاء الاصطناعي',
    time: 'الوقت',
    date: 'التاريخ',
    timezone: 'المنطقة الزمنية',
    weather: 'الطقس',
    temperature: 'درجة الحرارة',
    humidity: 'الرطوبة',
    windSpeed: 'سرعة الرياح',
    forecast: 'التوقعات',
    today: 'اليوم',
    tomorrow: 'غداً',
    currency: 'العملة',
    currencyConverter: 'محول العملات',
    from: 'من',
    to: 'إلى',
    amount: 'المبلغ',
    convert: 'تحويل',
    exchangeRate: 'سعر الصرف',
    settings: 'الإعدادات',
    language: 'اللغة',
    location: 'الموقع',
    preferences: 'التفضيلات',
    loading: 'جاري التحميل...',
    error: 'خطأ',
    success: 'نجاح',
    close: 'إغلاق',
    save: 'حفظ',
    cancel: 'إلغاء',
    monday: 'الاثنين',
    tuesday: 'الثلاثاء',
    wednesday: 'الأربعاء',
    thursday: 'الخميس',
    friday: 'الجمعة',
    saturday: 'السبت',
    sunday: 'الأحد',
    january: 'يناير',
    february: 'فبراير',
    march: 'مارس',
    april: 'أبريل',
    may: 'مايو',
    june: 'يونيو',
    july: 'يوليو',
    august: 'أغسطس',
    september: 'سبتمبر',
    october: 'أكتوبر',
    november: 'نوفمبر',
    december: 'ديسمبر',
  },
  pt: {
    workspace: 'Espaço de trabalho',
    rosLearning: 'Aprendizado ROS',
    fullIDE: 'IDE Completo',
    downloads: 'Downloads',
    documentation: 'Documentação',
    aiAssistant: 'Assistente IA',
    time: 'Hora',
    date: 'Data',
    timezone: 'Fuso horário',
    weather: 'Clima',
    temperature: 'Temperatura',
    humidity: 'Umidade',
    windSpeed: 'Velocidade do vento',
    forecast: 'Previsão',
    today: 'Hoje',
    tomorrow: 'Amanhã',
    currency: 'Moeda',
    currencyConverter: 'Conversor de moeda',
    from: 'De',
    to: 'Para',
    amount: 'Quantia',
    convert: 'Converter',
    exchangeRate: 'Taxa de câmbio',
    settings: 'Configurações',
    language: 'Idioma',
    location: 'Localização',
    preferences: 'Preferências',
    loading: 'Carregando...',
    error: 'Erro',
    success: 'Sucesso',
    close: 'Fechar',
    save: 'Salvar',
    cancel: 'Cancelar',
    monday: 'Segunda-feira',
    tuesday: 'Terça-feira',
    wednesday: 'Quarta-feira',
    thursday: 'Quinta-feira',
    friday: 'Sexta-feira',
    saturday: 'Sábado',
    sunday: 'Domingo',
    january: 'Janeiro',
    february: 'Fevereiro',
    march: 'Março',
    april: 'Abril',
    may: 'Maio',
    june: 'Junho',
    july: 'Julho',
    august: 'Agosto',
    september: 'Setembro',
    october: 'Outubro',
    november: 'Novembro',
    december: 'Dezembro',
  },
  ru: {
    workspace: 'Рабочее пространство',
    rosLearning: 'Обучение ROS',
    fullIDE: 'Полная IDE',
    downloads: 'Загрузки',
    documentation: 'Документация',
    aiAssistant: 'ИИ Ассистент',
    time: 'Время',
    date: 'Дата',
    timezone: 'Часовой пояс',
    weather: 'Погода',
    temperature: 'Температура',
    humidity: 'Влажность',
    windSpeed: 'Скорость ветра',
    forecast: 'Прогноз',
    today: 'Сегодня',
    tomorrow: 'Завтра',
    currency: 'Валюта',
    currencyConverter: 'Конвертер валют',
    from: 'Из',
    to: 'В',
    amount: 'Сумма',
    convert: 'Конвертировать',
    exchangeRate: 'Обменный курс',
    settings: 'Настройки',
    language: 'Язык',
    location: 'Местоположение',
    preferences: 'Предпочтения',
    loading: 'Загрузка...',
    error: 'Ошибка',
    success: 'Успех',
    close: 'Закрыть',
    save: 'Сохранить',
    cancel: 'Отмена',
    monday: 'Понедельник',
    tuesday: 'Вторник',
    wednesday: 'Среда',
    thursday: 'Четверг',
    friday: 'Пятница',
    saturday: 'Суббота',
    sunday: 'Воскресенье',
    january: 'Январь',
    february: 'Февраль',
    march: 'Март',
    april: 'Апрель',
    may: 'Май',
    june: 'Июнь',
    july: 'Июль',
    august: 'Август',
    september: 'Сентябрь',
    october: 'Октябрь',
    november: 'Ноябрь',
    december: 'Декабрь',
  },
};

export const languageNames: Record<string, { native: string; flag: string }> = {
  en: { native: 'English', flag: '🇬🇧' },
  es: { native: 'Español', flag: '🇪🇸' },
  fr: { native: 'Français', flag: '🇫🇷' },
  de: { native: 'Deutsch', flag: '🇩🇪' },
  zh: { native: '中文', flag: '🇨🇳' },
  ja: { native: '日本語', flag: '🇯🇵' },
  ar: { native: 'العربية', flag: '🇸🇦' },
  pt: { native: 'Português', flag: '🇵🇹' },
  ru: { native: 'Русский', flag: '🇷🇺' },
};

class LocalizationManager {
  private currentLanguage: string = 'en';
  private listeners: Array<() => void> = [];

  constructor() {
    this.detectLanguage();
  }

  private detectLanguage(): void {
    const saved = localStorage.getItem('navl_language');
    if (saved && translations[saved]) {
      this.currentLanguage = saved;
      return;
    }

    const browserLang = navigator.language.split('-')[0];
    if (translations[browserLang]) {
      this.currentLanguage = browserLang;
    }
  }

  setLanguage(lang: string): void {
    if (translations[lang]) {
      this.currentLanguage = lang;
      localStorage.setItem('navl_language', lang);
      this.notifyListeners();
    }
  }

  getLanguage(): string {
    return this.currentLanguage;
  }

  t(key: keyof LocalizationStrings): string {
    return translations[this.currentLanguage][key] || translations.en[key];
  }

  onChange(callback: () => void): void {
    this.listeners.push(callback);
  }

  private notifyListeners(): void {
    this.listeners.forEach(callback => callback());
  }
}

export const i18n = new LocalizationManager();
