source "https://rubygems.org"

# Hello! This is where you manage which Jekyll version is used to run.
# When you want to use a different version, change it below, save the
# file and run `bundle install`. Run Jekyll with `bundle exec`, like so:
#
#     bundle exec jekyll serve
#
# This will help ensure the proper Jekyll version is running.
# Happy Jekylling!
gem "jekyll", "~> 3.8"

# Fixes `jekyll serve` in ruby 3
gem "webrick"

group :jekyll_plugins do
  gem "github-pages", group: :jekyll_plugins
  gem "jekyll-include-cache", group: :jekyll_plugins
  gem "jekyll-compose"
end

gem 'wdm' if Gem.win_platform?
gem "tzinfo-data" if Gem.win_platform?
gem "faraday-retry", "~> 2.2"
