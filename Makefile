JAVA_HOME ?= /usr/lib/jvm/java-11-openjdk-amd64
export JAVA_HOME
MVN ?= ./mvnw

.PHONY: local test package up down logs demo wrapper contest

wrapper:
	mvn -N wrapper:wrapper -Dmaven=3.8.8

local:
	$(MVN) spring-boot:run -Dspring-boot.run.profiles=local

test:
	$(MVN) test

package:
	$(MVN) -DskipTests package

contest: package
	$(JAVA_HOME)/bin/java -jar target/heatnet.jar --process-contest --out samples/contest-result.geojson

up:
	docker-compose up --build

down:
	docker-compose down

logs:
	docker-compose logs -f app
