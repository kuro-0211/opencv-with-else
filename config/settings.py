"""Minimal Django settings for the vision monitor dashboard."""
from pathlib import Path

BASE_DIR = Path(__file__).resolve().parent.parent

# Directory shared with the ROS2 container (mounted as ./shared)
SHARED_DIR = BASE_DIR / "shared"

# Name of the running ROS2 container, used by /set_mode (docker exec ...)
ROS2_CONTAINER = "ros2_vision"

SECRET_KEY = "django-insecure-vision-monitor-demo-key-change-me"
DEBUG = True
ALLOWED_HOSTS = ["*"]

INSTALLED_APPS = [
    "django.contrib.contenttypes",
    "django.contrib.staticfiles",
    "monitor",
]

MIDDLEWARE = [
    "django.middleware.common.CommonMiddleware",
]

ROOT_URLCONF = "config.urls"

TEMPLATES = [
    {
        "BACKEND": "django.template.backends.django.DjangoTemplates",
        "DIRS": [],
        "APP_DIRS": True,
        "OPTIONS": {"context_processors": []},
    },
]

WSGI_APPLICATION = "config.wsgi.application"

# Django's own bookkeeping DB (separate from the ROS2 detections.db)
DATABASES = {
    "default": {
        "ENGINE": "django.db.backends.sqlite3",
        "NAME": BASE_DIR / "db.sqlite3",
    }
}

STATIC_URL = "static/"

DEFAULT_AUTO_FIELD = "django.db.models.BigAutoField"

USE_TZ = True
