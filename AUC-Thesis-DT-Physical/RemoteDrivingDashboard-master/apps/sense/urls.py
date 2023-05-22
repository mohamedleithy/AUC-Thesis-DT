from apps.sense import views
from django.conf.urls import url

urlpatterns = [
    url(r'^$', views.main , name='main'),
    url(r'^show_data/', views.show_data , name='show_data'),
    url(r'^real_data/',views.real_data,name='real_data'),
    url(r'^anomaly_detection/', views.anomaly_detection, name='anomaly_detection'),  # todo

]
