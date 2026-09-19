#!/usr/bin/env bash
set -euo pipefail

host="${DB_HOST:-db}"
port="${DB_PORT:-5432}"

echo "Ожидание PostgreSQL ${host}:${port}..."
for _ in $(seq 1 60); do
  if bash -c "echo > /dev/tcp/${host}/${port}" 2>/dev/null; then
    echo "PostgreSQL доступен."
    exec java ${JAVA_OPTS:-} -jar /app/app.jar
  fi
  sleep 2
done

echo "Не удалось дождаться PostgreSQL за 120 секунд." >&2
exit 1
