# encoding: utf-8

# -*- mode: ruby -*-
# vi: set ft=ruby :

Vagrant::Config.run do |config|
  config.vm.box = "precise64"
  config.vm.box_url = "http://files.vagrantup.com/precise64.box"
  config.vm.customize ["modifyvm", :id, "--memory", 8000]
  config.vm.customize ["modifyvm", :id, "--cpus", 2]

  config.vm.provision :shell, :path => "docs/install-flyvr.sh"

end
