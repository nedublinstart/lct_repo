JAVA_HOME ?= /usr/lib/jvm/java-11-openjdk-amd64
export JAVA_HOME
MVN ?= ./mvnw

.PHONY: local test package up down logs demo wrapper

wrapper:
	mvn -N wrapper:wrapper -Dmaven=3.8.8

local:
	$(MVN) spring-boot:run -Dspring-boot.run.profiles=local

test:
	$(MVN) test

package:
	$(MVN) -DskipTests package

up:
	docker-compose up --build

down:
	docker-compose down

logs:
	docker-compose logs -f app
