from django.urls import path

from monitor import views

urlpatterns = [
    path("", views.index, name="index"),
    path("api/state", views.api_state, name="api_state"),
    path("api/logs", views.api_logs, name="api_logs"),
    path("api/history", views.api_history, name="api_history"),
    path("video_feed", views.video_feed, name="video_feed"),
    path("set_mode", views.set_mode, name="set_mode"),
]
