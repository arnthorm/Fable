#!/usr/bin/env python

import roslib
roslib.load_manifest('posture_detection')
import rospy

from user_tracker.msg import UserAng
from posture_detection.node import PostureNode
from posture_detection.manager import PostureManager

import os
import sys
import cPickle as pickle
import pylab

def sum_list(values):
  result = 0
  for value in values:
    result += value
  return result


class PostureTest:
  def __init__(self, database_path='testdp.p'):
    rospy.init_node('posture_test', anonymous=True)
    self.database_path = database_path
    self.subscriptions = []
    self.args = {}
    self.package_path = roslib.packages.get_pkg_dir('posture_detection')
    self.manager = PostureManager(
      '%s/db.p' %
      roslib.packages.get_pkg_dir('posture_detection')
    )
    self._load_database()

  def _load_database(self):
    if os.path.exists(self.database_path):
      self.database = pickle.load(open(self.database_path, "rb"))
    else:
      self.database = []
    rospy.loginfo("Loaded %d test postures" % len(self.database))

  def _save_database(self):
    """Save current database to file (using Pickle)."""
    pickle.dump(self.database, open(self.database_path, "wb"))

  def _remove_subscriptions(self):
    for sub in self.subscriptions:
      sub.unregister()

  def add(self, *args, **kwargs):
    for i, item in enumerate(self.database):
      if item['username'] == kwargs['username'] and \
         item['posture'] == kwargs['posture'] and \
         item['set'] == kwargs['set']:
        print 'This has already been added to the database!'
        return

    self.args = kwargs
    self.subscriptions.append(
      rospy.Subscriber('user_ang', UserAng, self.add_cb)
    )
    rospy.spin()

  def list(self):
    for item in self.database:
      item['detected'] = self.manager.detect_posture(item['message'])
      del item['message']
      print item

  def remove(self, username, posture, set):
    for i, item in enumerate(self.database):
      if item['username'] == username and item['posture'] == posture and \
         item['set'] == set:
        rospy.loginfo("Deleted");
        del self.database[i]
      self._save_database()
      rospy.signal_shutdown('')

  def add_cb(self, data):
    self.args.update({'message': data})
    self.database.append(self.args)
    self._save_database()
    self._remove_subscriptions()
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

  def compare(self, group_by='posture', set=None, label=''):
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

if __name__ == '__main__':
  pt = PostureTest('%s/testdb.p' % roslib.packages.get_pkg_dir('posture_detection'))
  if len(sys.argv) > 1:
    command = sys.argv[1]
    if command == 'add' and len(sys.argv) == 5:
      # add username posture set
      pt.add(username=sys.argv[2], posture=sys.argv[3], set=sys.argv[4])
    elif command == 'remove' and len(sys.argv) == 5:
      pt.remove(username=sys.argv[2], posture=sys.argv[3], set=sys.argv[4])
    elif command == 'compare' and len(sys.argv) >= 2:
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
    elif command == 'edit':
      pt.edit()

