#!/usr/bin/env python

import roslib
roslib.load_manifest('user_tracker')
import rospy

from user_tracker.msg import UserPos, UserAng
from recognition.srv import RectFromTopic, InsertImage, RecognizeImageTest
from user_tracker.helpers import get_rect_from_point, get_point
from user_tracker.helpers import calc_distance, get_attr
from user_tracker.user_recognition import MAX_HEAD_WIDTH, MAX_HEAD_HEIGHT

import cv
from cv_bridge import CvBridge, CvBridgeError

import os
import sys
import cPickle as pickle
import pylab

EXT = '.png'

WINDOW = 'Image'

def sum_list(values):
  result = 0
  for value in values:
    result += value
  return result

GET_RECT_SERVICE = '/camera/rgb/image_color/get_rect'

def within(value, goal, pm):
  if goal - pm < value and value < goal + pm:
    return True
  else:
    return False

def find_filename(path, filename, ext, num=None):
  snum = '_0'
  if num is not None:
    snum = '_' + str(num)
  else:
    num = 0
  if os.path.exists(os.path.join(path, filename+snum+ext)):
    return find_filename(path, filename, ext, num+1)
  else:
    return (num, filename+snum)

def compare_keys(d1, d2):
  for key in d1.keys():
    if d1[key] != d2[key]:
      return False
  return True

def filter(db, **kwargs):
  l = []
  for entry in db:
    if compare_keys(kwargs, entry):
      l.append(entry)
  return l

def sum_list(l, field, group_by):
  group = {}
  for entry in l:
    if not group.has_key(entry[group_by]):
      group[entry[group_by]] = 0
    group[entry[group_by]] += entry[field]
  return group


class RecognitionTest:
  def __init__(self, database_path='testdp.p'):
    rospy.init_node('recognition_test', anonymous=True)
    self.database_path = database_path
    self.subscriptions = []
    self.args = {}
    self.bridge = CvBridge()
    self.package_path = roslib.packages.get_pkg_dir('user_tracker')
    self.get_rect = rospy.ServiceProxy(GET_RECT_SERVICE, RectFromTopic)
    self._load_database()

  def _load_database(self):
    if os.path.exists(self.database_path):
      self.database = pickle.load(open(self.database_path, "rb"))
    else:
      self.database = []
    rospy.loginfo("Loaded %d entries." % len(self.database))

  def _save_database(self):
    """Save current database to file (using Pickle)."""
    pickle.dump(self.database, open(self.database_path, "wb"))

  def _remove_subscriptions(self):
    for sub in self.subscriptions:
      sub.unregister()

  def add(self, *args, **kwargs):
    #for i, item in enumerate(self.database):
      #if item['username'] == kwargs['username'] and \
      #   item['posture'] == kwargs['posture'] and \
      #   item['set'] == kwargs['set']:
      #  print 'This has already been added to the database!'
      #  return

    self.args = kwargs
    self.subscriptions.append(
      rospy.Subscriber('user_pos', UserPos, self.add_cb)
    )
    cv.NamedWindow(WINDOW)
    self.cur = None
    if self.args['test'] == 'angle':
      self.subscriptions.append(
        rospy.Subscriber('user_ang', UserAng, self.user_ang_bogus)
      )
      self.dx = 5
      self.pm = 1
      self.first = 0
      self.last = 50
      self.field = 'body.angle'
      self.conv_func = lambda x: 180-x
    elif self.args['test'] == 'distance':
      self.dx = 0.2
      self.pm = 0.01
      self.first = 0.6
      self.last = 2.6
      self.field = 'head.position'
      self.conv_func = calc_distance

    rospy.spin()

  def user_ang_bogus(self, data):
    pass

  def take_picture(self, data):
    rect = get_rect_from_point(
      data.head.position,
      MAX_HEAD_WIDTH,
      MAX_HEAD_HEIGHT
    )
    self.args['pos'] = get_point(data.head.position)
    response = self.get_rect(*rect)
    (self.args['num'], self.args['file_path']) = self.save_img(
      response,
      '%s/img/%s' % (
        self.package_path,
        self.args['username'] 
      ),
      '%s_%s_%s' % (
        self.args['test'],
        self.args['set'],
        self.args['label']
      )
    )

    self.database.append(self.args)
    self._save_database()
    if self.args.has_key('value'):
      rospy.loginfo("Saved %s" % str(self.args['value']))
    else:
      rospy.loginfo("Saved")

  def add_cb(self, data):
    if self.args['test'] in ('angle', 'distance'):
      value = self.conv_func(get_attr(data, self.field))
      self.args['value'] = value
      if self.cur is None and (within(value, self.first, self.pm) or \
                              self.args['test'] == 'angle'):
        self.cur = self.first
        self.args['label'] = str(self.cur)
        self.take_picture(data)
      elif self.cur is not None:
        if within(value, self.cur + self.dx, self.pm) and \
           value > self.last:
          self.args['label'] = str(self.last)
          self.take_picture(data)
          self._remove_subscriptions()
          rospy.signal_shutdown('')
        elif within(value, self.cur + self.dx, self.pm):
          self.cur = self.cur + self.dx
          self.args['label'] = str(self.cur)
          self.take_picture(data)
    else:
      self.take_picture(data)
      self._remove_subscriptions()
      cv.WaitKey(0)
      rospy.signal_shutdown('')


  def save_img(self, response, path, filename):
    (num, filename) = find_filename(path, filename, EXT)
    file_path = os.path.join(path, filename)
    if not os.path.exists(path):
      os.makedirs(path)

    try:
      cv_image = self.bridge.imgmsg_to_cv(response.image, "bgr8")
      cv.ShowImage(WINDOW, cv_image)
      cv.SaveImage(file_path+EXT, cv_image)
      if response.mask.width > 0:
        cv_mask = self.bridge.imgmsg_to_cv(response.mask, "mono8")
        cv.SaveImage(file_path+'_mask'+EXT, cv_mask)
    except CvBridgeError, e:
      print e
    return (num, file_path)

  def list(self):
    for item in self.database:
      item['distance'] = calc_distance(item['pos'])
      #del item['pos']
      #del item['file_path']
      print item

  def remove(self):
    to_remove = []
    for i, item in enumerate(self.database):
      if not os.path.exists(item['file_path']+EXT):
        print item
        to_remove.append(i)

    if to_remove:
      rospy.loginfo("Removing %d entries." % len(to_remove));
      for i in reversed(to_remove):
        del self.database[i]
      self._save_database()
    else:
      rospy.loginfo("Nothing to remove.")
    rospy.signal_shutdown('')

  def edit(self):
    edit = False
    for i, item in enumerate(self.database):
      if item['username'] == 'arnthor' and item['posture'] == 'hands_up' and \
         item['set'] == '1':
        #self.database[i]['set'] = '2'
        edit = True
        rospy.loginfo("Edited.");
        break
    if edit:
      self._save_database()

  def load_image(self, entry):
    try:
      cv_image = cv.LoadImage(entry['file_path']+EXT)
      cv_mask = cv.LoadImage(entry['file_path']+'_mask'+EXT,
                             cv.CV_LOAD_IMAGE_GRAYSCALE)
      image = self.bridge.cv_to_imgmsg(cv_image, "bgr8")
      mask = self.bridge.cv_to_imgmsg(cv_mask, "mono8")
      return (image, mask)
    except CvBridgeError, e:
      print e
      raise

  def load(self):
    l = filter(self.database, test='front', set='1')
    insert_image = rospy.ServiceProxy('/recognize/test/insert_image',
                                      InsertImage)
    for entry in l:
      print entry
      if True:
        (image, mask) = self.load_image(entry)
        insert_image(image, mask, entry['username'])

  def compare(self):
    recognize_image = rospy.ServiceProxy('/recognize/test/image_test',
                                         RecognizeImageTest)
    results = []
    total = {}
    for entry in filter(self.database, username='sigga', test='angle',
                        label='0'):
      #print entry

      # Max keypoints / front compare
      if entry['username'] not in total.keys():
        front = filter(self.database, username=entry['username'], 
               test='front')[0]
        (image_front, mask_front) = self.load_image(front)
        response = recognize_image(image_front, mask_front)
        print 'Testnumber %d' % response.testNumber
        total[entry['username']] = response.testNumber

      (image, mask) = self.load_image(entry)
      response = recognize_image(image, mask)
      print response.testNumber
      if response.label.data and entry['username'] != response.label.data:
        print 'False positive'
      results.append({
        'detected': 1 if response.label == entry['username'] else 0,
        'matches': response.testNumber,
        'group_by': entry['label']
      })
      print 'Recognized: ' + str(response.label.data)

    sum_list(results, field='detected', group_by='group_by')
    sum_list(results, field='matches', group_by='group_by')

  def compare2(self, group_by='posture', set=None, label=''):
    results = {}
    if label:
      label = label + '_'
    for item in self.database:
      if set is not None and item['set'] == set or set is None:
        detected = self.manager.detect_posture(item['message'])
        detected = detected[0]
        value = 0
        if item['posture'] == detected:
          value = 1
          #print 'True positive'
        elif detected is not None:
          print 'False positive'
        else:
          pass
          #print 'No detection'
        if not results.has_key(item[group_by]):
          results[item[group_by]] = []
        results[item[group_by]].append(value)
    success_rate = {}
    for key in results:
      success_rate[key] = 100.0*sum_list(results[key])/len(results[key]) 
      print("%s: %f (%d/%d)" % (
        key,
        success_rate[key],
        sum_list(results[key]),
        len(results[key])
      ))
    pylab.ion()
    pylab.bar(range(0, len(success_rate)), success_rate.values())
    pylab.xticks(range(0, len(success_rate)), success_rate.keys(), rotation=17)
    pylab.ylabel("Success rate [%]")
    pylab.show()
    pylab.savefig('%s/img/%s%s_%s.pdf' % (self.package_path, label, group_by, set))
    ble = raw_input('Press enter to continue')

  def backup(self):
    import shutil
    import datetime
    backup_dir = os.path.join(self.package_path, 'backup')
    if not os.path.exists(backup_dir):
      os.makedirs(backup_dir)
    filename = datetime.datetime.now().strftime('%Y-%m-%d_%H-%M')+'.p'
    shutil.copy(self.database_path, '%s/%s' % (backup_dir, filename))

if __name__ == '__main__':
  pt = RecognitionTest('%s/testdb.p' %
                       roslib.packages.get_pkg_dir('user_tracker'))
  if len(sys.argv) > 1:
    command = sys.argv[1]
    if command == 'add' and len(sys.argv) >= 5:
      args = sys.argv
      label = ''
      if len(args) == 6:
        label = args[5]
      pt.add(username=args[2], test=args[3], set=args[4], label=label)
    elif command == 'remove' and len(sys.argv) == 2:
      pt.remove()
    elif command == 'compare':
      pt.compare()
    elif command == 'compare2' and len(sys.argv) >= 2:
      group_by = 'posture'
      set = None
      label = ''
      if len(sys.argv) > 2:
        group_by = sys.argv[2]
      if len(sys.argv) > 3:
        set = sys.argv[3]
      if len(sys.argv) > 4:
        lable = sys.argv[4]
      pt.compare(group_by, set, label)
    elif command == 'list':
      pt.list()
    elif command == 'load':
      pt.load()
    elif command == 'backup':
      pt.backup()


    #elif command == 'edit':
    #  pt.edit()

