/****************************************************************************
** Meta object code from reading C++ file 'rgbsegmentation.h'
**
** Created by: The Qt Meta Object Compiler version 67 (Qt 5.9.2)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "../../src/rgbsegmentation.h"
#include <QtCore/qbytearray.h>
#include <QtCore/qmetatype.h>
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'rgbsegmentation.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 67
#error "This file was generated using the moc from 5.9.2. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
QT_WARNING_PUSH
QT_WARNING_DISABLE_DEPRECATED
struct qt_meta_stringdata_RGBSegmentation_t {
    QByteArrayData data[10];
    char stringdata0[136];
};
#define QT_MOC_LITERAL(idx, ofs, len) \
    Q_STATIC_BYTE_ARRAY_DATA_HEADER_INITIALIZER_WITH_OFFSET(len, \
    qptrdiff(offsetof(qt_meta_stringdata_RGBSegmentation_t, stringdata0) + ofs \
        - idx * sizeof(QByteArrayData)) \
    )
static const qt_meta_stringdata_RGBSegmentation_t qt_meta_stringdata_RGBSegmentation = {
    {
QT_MOC_LITERAL(0, 0, 15), // "RGBSegmentation"
QT_MOC_LITERAL(1, 16, 17), // "loadButtonPressed"
QT_MOC_LITERAL(2, 34, 0), // ""
QT_MOC_LITERAL(3, 35, 17), // "saveButtonPressed"
QT_MOC_LITERAL(4, 53, 15), // "spinBox1Changed"
QT_MOC_LITERAL(5, 69, 5), // "value"
QT_MOC_LITERAL(6, 75, 15), // "spinBox2Changed"
QT_MOC_LITERAL(7, 91, 15), // "spinBox3Changed"
QT_MOC_LITERAL(8, 107, 15), // "spinBox4Changed"
QT_MOC_LITERAL(9, 123, 12) // "segmentation"

    },
    "RGBSegmentation\0loadButtonPressed\0\0"
    "saveButtonPressed\0spinBox1Changed\0"
    "value\0spinBox2Changed\0spinBox3Changed\0"
    "spinBox4Changed\0segmentation"
};
#undef QT_MOC_LITERAL

static const uint qt_meta_data_RGBSegmentation[] = {

 // content:
       7,       // revision
       0,       // classname
       0,    0, // classinfo
       7,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       0,       // signalCount

 // slots: name, argc, parameters, tag, flags
       1,    0,   49,    2, 0x0a /* Public */,
       3,    0,   50,    2, 0x0a /* Public */,
       4,    1,   51,    2, 0x0a /* Public */,
       6,    1,   54,    2, 0x0a /* Public */,
       7,    1,   57,    2, 0x0a /* Public */,
       8,    1,   60,    2, 0x0a /* Public */,
       9,    0,   63,    2, 0x0a /* Public */,

 // slots: parameters
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void, QMetaType::Int,    5,
    QMetaType::Void, QMetaType::Int,    5,
    QMetaType::Void, QMetaType::Int,    5,
    QMetaType::Void, QMetaType::Int,    5,
    QMetaType::Void,

       0        // eod
};

void RGBSegmentation::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        RGBSegmentation *_t = static_cast<RGBSegmentation *>(_o);
        Q_UNUSED(_t)
        switch (_id) {
        case 0: _t->loadButtonPressed(); break;
        case 1: _t->saveButtonPressed(); break;
        case 2: _t->spinBox1Changed((*reinterpret_cast< int(*)>(_a[1]))); break;
        case 3: _t->spinBox2Changed((*reinterpret_cast< int(*)>(_a[1]))); break;
        case 4: _t->spinBox3Changed((*reinterpret_cast< int(*)>(_a[1]))); break;
        case 5: _t->spinBox4Changed((*reinterpret_cast< int(*)>(_a[1]))); break;
        case 6: _t->segmentation(); break;
        default: ;
        }
    }
}

const QMetaObject RGBSegmentation::staticMetaObject = {
    { &QMainWindow::staticMetaObject, qt_meta_stringdata_RGBSegmentation.data,
      qt_meta_data_RGBSegmentation,  qt_static_metacall, nullptr, nullptr}
};


const QMetaObject *RGBSegmentation::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->dynamicMetaObject() : &staticMetaObject;
}

void *RGBSegmentation::qt_metacast(const char *_clname)
{
    if (!_clname) return nullptr;
    if (!strcmp(_clname, qt_meta_stringdata_RGBSegmentation.stringdata0))
        return static_cast<void*>(this);
    return QMainWindow::qt_metacast(_clname);
}

int RGBSegmentation::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QMainWindow::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 7)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 7;
    } else if (_c == QMetaObject::RegisterMethodArgumentMetaType) {
        if (_id < 7)
            *reinterpret_cast<int*>(_a[0]) = -1;
        _id -= 7;
    }
    return _id;
}
QT_WARNING_POP
QT_END_MOC_NAMESPACE
