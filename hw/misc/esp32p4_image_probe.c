/*
 * ESP32-P4 camera sensor: a receiver that records what the sensor sends
 *
 * Copyright (c) 2026 Necessary Innovations AB
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 *
 * An esp32p4-image-probe takes an esp32p4-image-source's stream on the
 * port it names, as a camera input would, and writes each callback to a
 * text file as one line, with the virtual time the callback ran at: for
 * tests, and for seeing what a sensor's settings send.
 *
 *   -object esp32p4-image-probe,id=p0,source=cam0,port=csi,log=FILE
 *
 * The records, numbers in decimal unless written 0x:
 *
 *   attach running=0|1 now=NS
 *   stream on|off t=NS now=NS
 *   vsync frame=F active=0|1 t=NS now=NS
 *   line frame=F line=L hsync=NS,NS de=NS,NS epoch=NS clock=N pclk=HZ
 *        count=N now=NS data=HHHH...
 *   packet frame=F line=L vc=V dt=0xDT wc=N ecc=0xEE crc=0xCCCC sot=NS
 *          data=NS end=NS eot=NS now=NS payload=HH...
 *
 * (each on one line), "data" a line's samples as 4 hex digits each and
 * "payload" a long packet's bytes as 2, both left out with data=off.
 */

#include "qemu/osdep.h"
#include "qapi/error.h"
#include "qemu/module.h"
#include "qom/object_interfaces.h"
#include "hw/misc/esp32p4_image_source.h"

OBJECT_DECLARE_SIMPLE_TYPE(ESP32P4ImageProbe, ESP32P4_IMAGE_PROBE)

struct ESP32P4ImageProbe {
    Object parent_obj;

    ESP32P4ImageSource *source;
    /*
     * The source attached to, held apart from the link, which object
     * finalization lets go before the probe can detach.
     */
    ESP32P4ImageSource *attached;
    char *port;
    char *log;
    bool data;

    FILE *f;
};

static int64_t probe_now(void)
{
    return qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
}

static void probe_stream(void *opaque, bool on, int64_t when)
{
    ESP32P4ImageProbe *p = opaque;

    fprintf(p->f, "stream %s t=%" PRId64 " now=%" PRId64 "\n",
            on ? "on" : "off", when, probe_now());
    fflush(p->f);
}

static void probe_vsync(void *opaque, uint32_t frame, bool active,
                        int64_t when)
{
    ESP32P4ImageProbe *p = opaque;

    fprintf(p->f, "vsync frame=%u active=%d t=%" PRId64 " now=%" PRId64 "\n",
            frame, active, when, probe_now());
    fflush(p->f);
}

static void probe_dvp_line(void *opaque, const ESP32P4DvpLine *l)
{
    ESP32P4ImageProbe *p = opaque;
    uint32_t i;

    fprintf(p->f, "line frame=%u line=%u hsync=%" PRId64 ",%" PRId64
            " de=%" PRId64 ",%" PRId64 " epoch=%" PRId64 " clock=%" PRIu64
            " pclk=%" PRIu64 " count=%u now=%" PRId64, l->frame, l->line,
            l->hsync_ns, l->hsync_end_ns, l->de_ns, l->de_end_ns,
            l->epoch_ns, l->clock, l->pclk_hz, l->count, probe_now());
    if (p->data) {
        fputs(" data=", p->f);
        for (i = 0; i < l->count; i++) {
            fprintf(p->f, "%04x", l->samples[i]);
        }
    }
    fputc('\n', p->f);
    fflush(p->f);
}

static void probe_csi_packet(void *opaque, const ESP32P4CsiPacket *k)
{
    ESP32P4ImageProbe *p = opaque;
    uint32_t i;

    fprintf(p->f, "packet frame=%u line=%u vc=%u dt=0x%02x wc=%u ecc=0x%02x "
            "crc=0x%04x sot=%" PRId64 " data=%" PRId64 " end=%" PRId64
            " eot=%" PRId64 " now=%" PRId64, k->frame, k->line, k->vc, k->dt,
            k->wc, k->ecc, k->crc, k->sot_ns, k->data_ns, k->end_ns,
            k->eot_ns, probe_now());
    if (p->data && k->payload) {
        fputs(" payload=", p->f);
        for (i = 0; i < k->wc; i++) {
            fprintf(p->f, "%02x", k->payload[i]);
        }
    }
    fputc('\n', p->f);
    fflush(p->f);
}

static const ESP32P4ImageSinkOps probe_ops = {
    .stream = probe_stream,
    .vsync = probe_vsync,
    .dvp_line = probe_dvp_line,
    .csi_packet = probe_csi_packet,
};

static void probe_complete(UserCreatable *uc, Error **errp)
{
    ESP32P4ImageProbe *p = ESP32P4_IMAGE_PROBE(uc);
    ESP32P4ImagePort port;

    if (!p->source || !p->log || !p->port) {
        error_setg(errp, "esp32p4-image-probe: needs source, port and log");
        return;
    }
    if (!strcmp(p->port, "dvp")) {
        port = ESP32P4_IMAGE_PORT_DVP;
    } else if (!strcmp(p->port, "csi")) {
        port = ESP32P4_IMAGE_PORT_CSI;
    } else {
        error_setg(errp, "esp32p4-image-probe: port is dvp or csi");
        return;
    }
    p->f = fopen(p->log, "w");
    if (!p->f) {
        error_setg_errno(errp, errno, "esp32p4-image-probe: %s", p->log);
        return;
    }
    if (!esp32p4_image_source_bind(p->source, port, errp)) {
        return;
    }
    esp32p4_image_source_attach(p->source, &probe_ops, p);
    p->attached = ESP32P4_IMAGE_SOURCE(object_ref(OBJECT(p->source)));
    fprintf(p->f, "attach running=%d now=%" PRId64 "\n",
            esp32p4_image_source_streaming(p->source), probe_now());
    fflush(p->f);
}

static char *probe_get_port(Object *obj, Error **errp)
{
    return g_strdup(ESP32P4_IMAGE_PROBE(obj)->port);
}

static void probe_set_port(Object *obj, const char *value, Error **errp)
{
    ESP32P4ImageProbe *p = ESP32P4_IMAGE_PROBE(obj);

    g_free(p->port);
    p->port = g_strdup(value);
}

static char *probe_get_log(Object *obj, Error **errp)
{
    return g_strdup(ESP32P4_IMAGE_PROBE(obj)->log);
}

static void probe_set_log(Object *obj, const char *value, Error **errp)
{
    ESP32P4ImageProbe *p = ESP32P4_IMAGE_PROBE(obj);

    g_free(p->log);
    p->log = g_strdup(value);
}

static bool probe_get_data(Object *obj, Error **errp)
{
    return ESP32P4_IMAGE_PROBE(obj)->data;
}

static void probe_set_data(Object *obj, bool value, Error **errp)
{
    ESP32P4_IMAGE_PROBE(obj)->data = value;
}

static void probe_init(Object *obj)
{
    ESP32P4_IMAGE_PROBE(obj)->data = true;
}

static void probe_finalize(Object *obj)
{
    ESP32P4ImageProbe *p = ESP32P4_IMAGE_PROBE(obj);

    if (p->attached) {
        esp32p4_image_source_detach(p->attached, p);
        object_unref(OBJECT(p->attached));
    }
    if (p->f) {
        fclose(p->f);
    }
    g_free(p->port);
    g_free(p->log);
}

static void probe_class_init(ObjectClass *oc, const void *data)
{
    UserCreatableClass *ucc = USER_CREATABLE_CLASS(oc);

    ucc->complete = probe_complete;
    object_class_property_add_link(oc, "source", TYPE_ESP32P4_IMAGE_SOURCE,
                                   offsetof(ESP32P4ImageProbe, source),
                                   object_property_allow_set_link,
                                   OBJ_PROP_LINK_STRONG);
    object_class_property_set_description(oc, "source",
        "The esp32p4-image-source to listen to");
    object_class_property_add_str(oc, "port", probe_get_port,
                                  probe_set_port);
    object_class_property_set_description(oc, "port",
        "The port the sensor is wired to: dvp or csi");
    object_class_property_add_str(oc, "log", probe_get_log, probe_set_log);
    object_class_property_set_description(oc, "log",
        "The file the records go to");
    object_class_property_add_bool(oc, "data", probe_get_data,
                                   probe_set_data);
    object_class_property_set_description(oc, "data",
        "Record the lines' samples and the packets' payloads (default on)");
}

static const TypeInfo probe_types[] = {
    {
        .name = TYPE_ESP32P4_IMAGE_PROBE,
        .parent = TYPE_OBJECT,
        .instance_size = sizeof(ESP32P4ImageProbe),
        .instance_init = probe_init,
        .instance_finalize = probe_finalize,
        .class_init = probe_class_init,
        .interfaces = (const InterfaceInfo[]) {
            { TYPE_USER_CREATABLE },
            { }
        },
    },
};

DEFINE_TYPES(probe_types)
