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

#define LOG_TAG "StylisticButtons"

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

const int ID_ORIENTATION = (SENSORS_HANDLE_BASE + 0);
#define STYLISTIC_BUTTONS_HARDWARE_MODULE_ID SENSORS_HARDWARE_MODULE_ID
//#define STYLISTIC_BUTTONS_HARDWARE_MODULE_ID "sensors.stylisticbuttons"

template <typename T> struct SensorFd : T {
	int ufd;

	SensorFd(const struct hw_module_t *module, struct hw_device_t **device);
	~SensorFd();
};

template <typename T> SensorFd<T>::SensorFd(const struct hw_module_t *module, struct hw_device_t **device) : ufd(-1)
{
	this->common.tag     = HARDWARE_DEVICE_TAG;
	this->common.version = 0;
	this->common.module  = const_cast<struct hw_module_t *>(module);
	*device              = &this->common;
	LOGD("%s: module=%p dev=%p", __FUNCTION__, module, *device);
}

template <typename T> SensorFd<T>::~SensorFd()
{
	close(ufd);
}

struct SensorPollContext : SensorFd<sensors_poll_device_t> {
  public:
	SensorPollContext(const struct hw_module_t *module, struct hw_device_t **device);
	~SensorPollContext();

  private:
	static int poll_close(struct hw_device_t *dev);
	static int poll_activate(struct sensors_poll_device_t *dev, int handle, int enabled);
	static int poll_setDelay(struct sensors_poll_device_t *dev, int handle, int64_t ns);
	static int poll_poll(struct sensors_poll_device_t *dev, sensors_event_t *data, int count);

	enum {
		ROT_0,
		ROT_90,
		ROT_180,
		ROT_270
	};

	bool enabled;
	int rotation;
	struct timespec delay;
	struct pollfd pfd;
	sensors_event_t orients[4];
};

SensorPollContext::SensorPollContext(const struct hw_module_t *module, struct hw_device_t **device)
      : SensorFd<sensors_poll_device_t>(module, device), enabled(false), rotation(ROT_0)
{
	common.close = poll_close;
	activate     = poll_activate;
	setDelay     = poll_setDelay;
	poll         = poll_poll;

    int eventid = ROT_0;
	int &fd = pfd.fd;
	const char *dirname = "/dev/input";
	if (DIR *dir = opendir(dirname)) {
		while (struct dirent *de = readdir(dir)) {
            LOGD("Checking device: %s", de->d_name);
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

            LOGD("Considering opening Name: %s", name);

			// TODO: parse /etc/excluded-input-devices.xml
			if (!strcmp(name, "Fujitsu FUJ02BF")) {
				LOGI("open %s ok", name);
				break;
			}
			close(fd);
		}
		closedir(dir);
	}

	/*ufd = open("/dev/uinput", O_WRONLY | O_NDELAY);
	if (ufd >= 0) {
		struct uinput_user_dev ud;
		memset(&ud, 0, sizeof(ud));
		strcpy(ud.name, "Tega V2 Buttons");
		write(ufd, &ud, sizeof(ud));
		ioctl(ufd, UI_SET_EVBIT, EV_KEY);
		ioctl(ufd, UI_SET_EVBIT, EV_REP);
		ioctl(ufd, UI_SET_KEYBIT, KEY_ESC);
		ioctl(ufd, UI_SET_KEYBIT, KEY_COMPOSE);
		ioctl(ufd, UI_SET_KEYBIT, KEY_LEFTMETA);
		ioctl(ufd, UI_DEV_CREATE, 0);
	} else {
		LOGE("could not open uinput device: %s", strerror(errno));
	}*/

	pfd.events = POLLIN;

    // Initialize the orientation events.
    LOGD("Initializing: %d degrees.", (90 * eventid));
    orients[ROT_0].version             = sizeof(sensors_event_t);
    orients[ROT_0].sensor              = ID_ORIENTATION;
    orients[ROT_0].type                = SENSOR_TYPE_ACCELEROMETER;
    orients[ROT_0].orientation.status  = SENSOR_STATUS_ACCURACY_HIGH;
    orients[ROT_270] = orients[ROT_180] = orients[ROT_90] = orients[ROT_0];

    const double angle = 20.0;
    const double cos_angle = GRAVITY_EARTH * cos(angle / M_PI);
    const double sin_angle = GRAVITY_EARTH * sin(angle / M_PI);
    /*orients[ROT_0].acceleration.x   = 0.0;
    orients[ROT_0].acceleration.y   = cos_angle;
    orients[ROT_0].acceleration.z   = sin_angle;
    orients[ROT_90].acceleration.x  = cos_angle;
    orients[ROT_90].acceleration.y  = 0.0;
    orients[ROT_90].acceleration.z  = sin_angle;
    orients[ROT_180].acceleration.x = 0.0;
    orients[ROT_180].acceleration.y = -cos_angle;
    orients[ROT_180].acceleration.z = -sin_angle;
    orients[ROT_270].acceleration.x = -cos_angle;
    orients[ROT_270].acceleration.y = 0.0;
    orients[ROT_270].acceleration.z = -sin_angle;*/

    orients[ROT_0].acceleration.x = 0;
    orients[ROT_90].acceleration.x = 1;
    orients[ROT_180].acceleration.x = 2;
    orients[ROT_270].acceleration.x = 3;

	delay.tv_sec = 0;
	delay.tv_nsec = 300000000L;

	LOGD("%s: dev=%p ufd=%d fd=%d", __FUNCTION__, this, ufd, fd);
}

SensorPollContext::~SensorPollContext()
{
	close(pfd.fd);
}

int SensorPollContext::poll_close(struct hw_device_t *dev)
{
	LOGD("%s: dev=%p", __FUNCTION__, dev);
	delete reinterpret_cast<SensorPollContext *>(dev);
	return 0;
}
int SensorPollContext::poll_activate(struct sensors_poll_device_t *dev, int handle, int enabled)
{
	LOGD("%s: dev=%p handle=%d enabled=%d", __FUNCTION__, dev, handle, enabled);
	SensorPollContext *ctx = reinterpret_cast<SensorPollContext *>(dev);
	ctx->enabled = enabled;
	return 0;
}
int SensorPollContext::poll_setDelay(struct sensors_poll_device_t *dev, int handle, int64_t ns)
{
	LOGD("%s: dev=%p delay-ns=%lld", __FUNCTION__, dev, ns);
	return 0;
}
int SensorPollContext::poll_poll(struct sensors_poll_device_t *dev, sensors_event_t *data, int count)
{
	LOGD("%s: dev=%p data=%p count=%d", __FUNCTION__, dev, data, count);
	SensorPollContext *ctx = reinterpret_cast<SensorPollContext *>(dev);

	struct pollfd &pfd = ctx->pfd;
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
        if (iev.type == EV_KEY && iev.value) {
			int rot = ctx->rotation;
			switch (iev.code)
			{
                case 153:
                    LOGD("Rotate key pressed...");
                    LOGD("  Rotation:     %d", ctx->rotation);
                    LOGD("  ROT_0:        %d", ROT_0); 
                    LOGD("  ROT_90:       %d", ROT_90); 
                    LOGD("  ROT_180:      %d", ROT_180); 
                    LOGD("  ROT_270:      %d", ROT_270); 
                    /*if(ctx->rotation == ROT_0) {
                        rot = ROT_270;
                    } else {
                        rot = ROT_0;
                    }*/
                    rot++;
                    if(rot > ROT_270) rot = ROT_0;
                    LOGD("  New Rotation: %d", rot);
                    break;
                case 56:
                    LOGD("Home key pressed...");
                    iev.code = 125;
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
				if (rot != ctx->rotation) {
					LOGI("orientation changed from %d to %d", ctx->rotation * 90, rot * 90);
					ctx->rotation = rot;
				}
				if (ctx->enabled && count > 0)
					break;
			}
		}

		if (ctx->ufd >= 0)
			write(ctx->ufd, &iev, sizeof(iev));
	}

	LOGD("%s: dev=%p ufd=%d fd=%d rotation=%d", __FUNCTION__, dev, ctx->ufd, pfd.fd, ctx->rotation * 90);
	nanosleep(&ctx->delay, 0);
	data[0] = ctx->orients[ctx->rotation];
	data[0].timestamp = time(0) * 1000000000L;
	return 1;
}

static int open_kbd_sensor(const struct hw_module_t *module, const char *id, struct hw_device_t **device)
{
	LOGD("%s: id=%s", __FUNCTION__, id);
	return new SensorPollContext(module, device) ? 0 : -EINVAL;
}

static struct sensor_t sSensorListInit[] = {
	{
		name: "Stylistic Button Sensor",
		vendor: "Android-x86 Open Source Project",
		version: 1,
		handle: ID_ORIENTATION,
		type: SENSOR_TYPE_ACCELEROMETER,
		maxRange: 2.8f,
		resolution: 1.0f/4032.0f,
		power: 3.0f,
		minDelay: 0,
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
		version_minor: 3,
		id: STYLISTIC_BUTTONS_HARDWARE_MODULE_ID,
		name: "Stylistic Button Sensor",
		author: "Yolan",
		methods: &sensors_methods,
		dso: 0,
		reserved: { }
	},
	get_sensors_list: sensors_get_sensors_list
};
