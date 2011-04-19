/**
 *
 * Atkbd style sensor
 *
 * Copyright (C) 2011 The Android-x86 Open Source Project
 *
 * by Chih-Wei Huang <cwhuang@linux.org.tw>
 *
 * Licensed under GPLv2 or later
 *
 **/

#define LOG_TAG "KbdSensor"

#include <cmath>
#include <cerrno>
#include <cstring>
#include <sys/stat.h>
#include <poll.h>
#include <fcntl.h>
#include <dirent.h>
#include <cutils/log.h>
#include <linux/input.h>
#include <linux/uinput.h>
#include <hardware/sensors.h>

const int ID_ACCELERATION = (SENSORS_HANDLE_BASE + 0);

template <typename T> struct SensorFd : T {
	int ufd;

	SensorFd(const struct hw_module_t *module, struct hw_device_t **device);
	~SensorFd();

	static int common_close(struct hw_device_t *dev);
};

template <typename T> SensorFd<T>::SensorFd(const struct hw_module_t *module, struct hw_device_t **device) : ufd(-1)
{
	this->common.tag     = HARDWARE_DEVICE_TAG;
	this->common.version = 0;
	this->common.module  = const_cast<struct hw_module_t *>(module);
	this->common.close   = common_close;
	*device              = &this->common;
	LOGD("%s: module=%p dev=%p", __FUNCTION__, module, *device);
}

template <typename T> SensorFd<T>::~SensorFd()
{
	close(ufd);
}

template <typename T> int SensorFd<T>::common_close(struct hw_device_t *dev)
{
	LOGD("%s: dev=%p", __FUNCTION__, dev);
	delete reinterpret_cast<SensorFd<T> *>(dev);
	return 0;
}

/**
 ** SENSORS CONTROL DEVICE -- used to send commands to the sensors drivers
 **/
struct SensorControl : SensorFd<sensors_control_device_t> {
  public:
	SensorControl(const struct hw_module_t *module, struct hw_device_t **device);

  private:
	static native_handle_t *control_open_data_source(struct sensors_control_device_t *dev);
	static int control_activate(struct sensors_control_device_t *dev, int handle, int enabled);
	static int control_set_delay(struct sensors_control_device_t *dev, int32_t ms);
	static int control_wake(struct sensors_control_device_t *dev);
};

SensorControl::SensorControl(const struct hw_module_t *module, struct hw_device_t **device)
      : SensorFd<sensors_control_device_t>(module, device)
{
	open_data_source = control_open_data_source;
	activate         = control_activate;
	set_delay        = control_set_delay;
	wake             = control_wake;
}

native_handle_t* SensorControl::control_open_data_source(struct sensors_control_device_t *dev)
{
	SensorControl *ctl = reinterpret_cast<SensorControl *>(dev);
	native_handle_t *handle;
	handle = native_handle_create(1, 1);
	int fd = -1;
	const char *dirname = "/dev/input";
	if (DIR *dir = opendir(dirname)) {
		while (struct dirent *de = readdir(dir)) {
			if (de->d_name[0] != 'e') // eventX
				continue;
			char name[PATH_MAX];
			snprintf(name, PATH_MAX, "%s/%s", dirname, de->d_name);
			fd = open(name, O_RDWR);
			if (fd < 0) {
				LOGE("could not open %s, %s", name, strerror(errno));
				continue;
			}
			name[sizeof(name) - 1] = '\0';
			if (ioctl(fd, EVIOCGNAME(sizeof(name) - 1), &name) < 1) {
				LOGE("could not get device name for %s, %s\n", name, strerror(errno));
				name[0] = '\0';
			}

			// TODO: parse /etc/excluded-input-devices.xml
			if (!strcmp(name, "AT Translated Set 2 keyboard")) {
				LOGI("open %s ok", name);
				break;
			}
			close(fd);
		}
		closedir(dir);
	}
	handle->data[0] = fd;

	handle->data[1] = -1;
	if (ctl->ufd < 0) {
		fd = open("/dev/uinput", O_WRONLY | O_NDELAY);
		if (fd >= 0) {
			struct uinput_user_dev ud;
			memset(&ud, 0, sizeof(ud));
			strcpy(ud.name, "Tega V2 Buttons");
			write(fd, &ud, sizeof(ud));
			ioctl(fd, UI_SET_EVBIT, EV_KEY);
			ioctl(fd, UI_SET_EVBIT, EV_REP);
			ioctl(fd, UI_SET_KEYBIT, KEY_ESC);
			ioctl(fd, UI_SET_KEYBIT, KEY_COMPOSE);
			ioctl(fd, UI_SET_KEYBIT, KEY_LEFTMETA);
			ioctl(fd, UI_DEV_CREATE, 0);
		} else {
			LOGE("could not open uinput device: %s", strerror(errno));
		}
		handle->data[1] = ctl->ufd = fd;
	}

	LOGD("%s: dev=%p handle=%p data[0]=%d data[1]=%d", __FUNCTION__, dev, handle, handle->data[0], handle->data[1]);
	return handle;
}

int SensorControl::control_activate(struct sensors_control_device_t *dev, int handle, int enabled)
{
	LOGD("%s: dev=%p handle=%d enabled=%d", __FUNCTION__, dev, handle, enabled);
	return 0;
}

int SensorControl::control_set_delay(struct sensors_control_device_t *dev, int32_t ms)
{
	LOGD("%s: dev=%p delay-ms=%d", __FUNCTION__, dev, ms);
	return 0;
}

int SensorControl::control_wake(struct sensors_control_device_t *dev)
{
	LOGD("%s: dev=%p", __FUNCTION__, dev);
	return 0;
}

/**
 ** SENSORS DATA DEVICE -- used to read sensor data from the hardware.
 **/
class SensorData : SensorFd<sensors_data_device_t> {
  public:
	SensorData(const struct hw_module_t *module, struct hw_device_t **device);

  private:
	static int data_data_open(struct sensors_data_device_t *dev, native_handle_t *handle);
	static int data_data_close(struct sensors_data_device_t *dev);
	static int data_poll(struct sensors_data_device_t *dev, sensors_data_t *values);

	enum {
		ROT_0,
		ROT_90,
		ROT_180,
		ROT_270
	};

	int rotation;
	struct pollfd pfd;
	sensors_data_t orients[4];
};

SensorData::SensorData(const struct hw_module_t *module, struct hw_device_t **device)
      : SensorFd<sensors_data_device_t>(module, device), rotation(ROT_0)
{
	data_open      = data_data_open;
	data_close     = data_data_close;
	poll           = data_poll;

	pfd.events = POLLIN;
	orients[ROT_0].sensor = ID_ACCELERATION;
	orients[ROT_0].acceleration.status = SENSOR_STATUS_ACCURACY_HIGH;
	orients[ROT_270] = orients[ROT_180] = orients[ROT_90] = orients[ROT_0];
	const double angle = 20.0;
	const double cos_angle = GRAVITY_EARTH * cos(angle / M_PI);
	const double sin_angle = GRAVITY_EARTH * sin(angle / M_PI);
	orients[ROT_0].acceleration.x   = 0.0;
	orients[ROT_0].acceleration.y   = cos_angle;
	orients[ROT_0].acceleration.z   = sin_angle;
	orients[ROT_90].acceleration.x  = cos_angle;
	orients[ROT_90].acceleration.y  = 0.0;
	orients[ROT_90].acceleration.z  = sin_angle;
	orients[ROT_180].acceleration.x = 0.0;
	orients[ROT_180].acceleration.y = +cos_angle;
	orients[ROT_180].acceleration.z = -sin_angle;
	orients[ROT_270].acceleration.x = -cos_angle;
	orients[ROT_270].acceleration.y = 0.0;
	orients[ROT_270].acceleration.z = -sin_angle;
}

int SensorData::data_data_open(struct sensors_data_device_t *dev, native_handle_t *handle)
{
	SensorData *data = reinterpret_cast<SensorData *>(dev);

	data->ufd = handle->data[1];
	data->pfd.fd = dup(handle->data[0]);
	LOGD("%s: dev=%p ufd=%d fd=%d(%d) handle=%p)", __FUNCTION__, dev, data->ufd, data->pfd.fd, handle->data[0], handle);
	native_handle_close(handle);
	native_handle_delete(handle);
	return 0;
}

int SensorData::data_data_close(struct sensors_data_device_t *dev)
{
	LOGD("%s: dev=%p", __FUNCTION__, dev);
	SensorData *data = reinterpret_cast<SensorData *>(dev);
	if (data) {
		close(data->ufd);
		data->ufd = -1;
		close(data->pfd.fd);
		data->pfd.fd = -1;
	}
	return 0;
}

int SensorData::data_poll(struct sensors_data_device_t *dev, sensors_data_t *values)
{
	SensorData *data = reinterpret_cast<SensorData *>(dev);
	LOGV("%s: dev=%p fd=%d,%d", __FUNCTION__, dev, data->fd[0], data->fd[1]);

	struct pollfd &pfd = data->pfd;
	while (int pollres = ::poll(&pfd, 1, -1)) {
		if (pollres < 0) {
			LOGE("%s: poll %d error: %s", __FUNCTION__, pfd.fd, strerror(errno));
			break;
		}
		if (!(pfd.revents & POLLIN)) {
			LOGW("%s: ignore revents %d", __FUNCTION__, pfd.revents);
			continue;
		}

		struct input_event iev;
		size_t res = ::read(pfd.fd, &iev, sizeof(iev));
		if (res < sizeof(iev)) {
			LOGW("insufficient input data(%d)? fd=%d", res, pfd.fd);
			continue;
		}
		LOGD("type=%d scancode=%d value=%d from fd=%d", iev.type, iev.code, iev.value, pfd.fd);
		if (iev.type == EV_KEY) {
			int rot = -1;
			switch (iev.code)
			{
				case KEY_LEFTCTRL:
				case KEY_LEFTALT:
					if (iev.value)
						continue;
					rot = data->rotation;
					break;
				case KEY_UP:
					rot = ROT_0;
					break;
				case KEY_RIGHT:
					rot = ROT_90;
					break;
				case KEY_DOWN:
					rot = ROT_180;
					break;
				case KEY_LEFT:
					rot = ROT_270;
					break;
#if 0
				case KEY_ESC:
					iev.code = KEY_LEFTMETA;
					break;
				case KEY_COMPOSE:
					iev.code = KEY_ESC;
					break;
#endif
			}
			if (rot >= 0) {
				if (rot != data->rotation) {
					LOGI("orientation changed from %d to %d", data->rotation * 90, rot * 90);
					data->rotation = rot;
				}
				break;
			}
		}

		if (data->ufd >= 0)
			write(data->ufd, &iev, sizeof(iev));
	}

	*values = data->orients[data->rotation];
	LOGD("%s: dev=%p ufd=%d fd=%d rotation=%d", __FUNCTION__, dev, data->ufd, pfd.fd, data->rotation * 90);
	return 0;
}

static int open_kbd_sensor(const struct hw_module_t *module, const char *id, struct hw_device_t **device)
{
	LOGD("%s: id=%s", __FUNCTION__, id);
	void *dev = 0;
	if (!strcmp(id, SENSORS_HARDWARE_CONTROL))
		dev = new SensorControl(module, device);
	else if (!strcmp(id, SENSORS_HARDWARE_DATA))
		dev = new SensorData(module, device);
	return dev ? 0 : -1;
}

static struct sensor_t sSensorListInit[] = {
	{
		name: "Kbd Orientation Sensor",
		vendor: "Android-x86 Open Source Project",
		version: 1,
		handle: ID_ACCELERATION,
		type: SENSOR_TYPE_ACCELEROMETER,
		maxRange: 2.8f,
		resolution: 1.0f/4032.0f,
		power: 3.0f,
		reserved: { }
	}
};

static int sensors_get_sensors_list(struct sensors_module_t *module, struct sensor_t const **list)
{
	*list = sSensorListInit;
	return sizeof(sSensorListInit) / sizeof(struct sensor_t);
}

static struct hw_module_methods_t sensors_methods = {
	open: open_kbd_sensor
};

struct sensors_module_t HAL_MODULE_INFO_SYM = {
	common: {
		tag: HARDWARE_MODULE_TAG,
		version_major: 2,
		version_minor: 2,
		id: SENSORS_HARDWARE_MODULE_ID,
		name: "Kbd Orientation Sensor",
		author: "Chih-Wei Huang",
		methods: &sensors_methods,
		dso: 0,
		reserved: { }
	},
	get_sensors_list: sensors_get_sensors_list
};
