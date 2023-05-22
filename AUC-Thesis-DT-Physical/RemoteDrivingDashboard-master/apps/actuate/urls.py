from apps.actuate import views
from django.conf.urls import url


urlpatterns = [
    url(r'^$', views.main , name='main'),
    url(r'^actuate_data/', views.actuate_data, name='actuate_data'),
    url(r'^steering_angle/', views.steering_angle, name='steering_angle'),
    url(r'^direction/', views.direction, name='direction'),

]